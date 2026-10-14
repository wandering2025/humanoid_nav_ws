"""Rebuild W13 reliability and performance reports from preserved run artifacts."""
import ast
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

WS=Path(__file__).resolve().parents[1]
# When preparing in /tmp pass workspace explicitly; installed script uses its parent.
if not (WS/'src').exists():
    WS=Path('/home/haoyang/humanoid_nav_ws')
sys.path.insert(0,str(WS/'scripts'))
from evaluation.metrics import classify, distribution, trace_columns


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_campaign(name):
    path=WS/'docs/evidence/W13'/name
    state=json.loads((path/'state.json').read_text())
    report=json.loads((path/'summary.json').read_text())
    results={}
    for row in report['rows']:
        p=path/(row.get('run_id','')+'.json')
        if p.exists(): results[row['case']]=json.loads(p.read_text())
    return state,report,results


def performance(name, results):
    pooled={key:[] for key in ('rtf','step_compute_ms','policy_compute_ms','physics_compute_ms','scan_compute_ms')}
    nav={}; sources=[]
    for result in results.values():
        run=WS/'run_data'/result['run_id']
        _,cols=trace_columns(run/'sim_trace.csv')
        for key in pooled:
            values=cols['rtf_10s'][(cols['wall_mono']-cols['wall_mono'][0])>=10] if key=='rtf' else cols[key][cols['scan_due']>0] if key=='scan_compute_ms' else cols[key]
            pooled[key].extend(values)
        cpu=next((WS/'docs/evidence/W13/raw'/name/result['run_id']).glob('*-cpu.json'))
        for sample in json.loads(cpu.read_text()):
            if result.get('nav2',{}).get('accepted_sim',float('inf')) <= sample['sim'] <= result.get('nav2',{}).get('end_sim',-float('inf')):
                nav.setdefault(sample['process'],[]).append(sample['cpu_ms_per_wall_s'])
        sources.append(dict(run_id=result['run_id'],trace_sha256=sha(run/'sim_trace.csv'),geometry_sha256=sha(run/'geometry.jsonl'),contacts_sha256=sha(run/'contact_events.csv'),cpu_sha256=sha(cpu)))
    return dict(samples={k:distribution(v) for k,v in pooled.items()},navigation_cpu_ms_per_wall_s={k:distribution(v) for k,v in nav.items()},sources=sources)



def protective_braking(result):
    """Additional offline braking observations, including slowdown -> gait deadzone.

    Formal success labels and the original CM-stop-only metrics are preserved.
    The added analysis is applied uniformly to every run after the batch.
    """
    _,tr=trace_columns(WS/'run_data'/result['run_id']/'sim_trace.csv')
    t,x,y=tr['sim_time'],tr['x_w'],tr['y_w'];yaw=np.unwrap(tr['yaw_w'])
    zero=(tr['u_pol_x']==0)&(tr['u_pol_y']==0)&(tr['u_pol_wz']==0)
    stationary=np.zeros(len(t),dtype=bool)
    stationary[25:]=(np.hypot(x[25:]-x[:-25],y[25:]-y[:-25])/.5<.05)&(abs(yaw[25:]-yaw[:-25])/.5<.05)
    out=[];seen=set();accepted=result.get('nav2',{}).get('accepted_sim',float('inf'))
    for e in result['run']['events']:
        if e['kind']!='collision_monitor' or e['action_type'] not in (1,2) or e['sim']<accepted:
            continue
        i=int(np.searchsorted(t,e['sim']))
        if i>=len(t) or not np.any(~zero[max(0,i-50):i]): continue
        candidates=np.flatnonzero(zero & (t>=e['sim']-.02) & (t<=e['sim']+.5))
        if not len(candidates): continue
        j=int(candidates[0])
        if j in seen: continue
        seen.add(j)
        resumed=np.flatnonzero(~zero[j:]);k=j+int(resumed[0]) if len(resumed) else len(t)
        stop=next((q for q in range(j,k-100) if stationary[q:q+101].all()),None)
        out.append(dict(run_id=result['run_id'],case=result['case_spec']['id'],cm_action=e['action_type'],
            reason='CM_STOP' if e['action_type']==1 else 'CM_SLOWDOWN_WITH_ZERO_POLICY',trigger_sim=e['sim'],
            trigger_speed_m_s=float(tr['v_meas_x'][i]),zero_policy_sim=float(t[j]),
            zero_latency_s=max(0.,float(t[j]-e['sim'])),max_displacement_after_zero_m=float(np.hypot(x[j:k]-x[j],y[j:k]-y[j]).max()),
            stationary_hold_2s_start_sim=float(t[stop]) if stop is not None else None,
            resumed_sim=float(t[k]) if k<len(t) else None))
    return out


def main():
    state,summary,runs=load_campaign('formal-v1')
    out=WS/'docs/results';out.mkdir(exist_ok=True)
    corrections=[]
    for row in summary['rows']:
        r=runs[row['case']]
        original=r['outcome']
        derived=classify(r,all(r.get('validity_checks',{}).values()))
        assert (original=='SUCCESS') == (derived=='SUCCESS'), 'Offline classification may not manufacture arrival'
        if derived!=original:
            corrections.append(dict(run_id=r['run_id'],original=original,derived=derived,
                input_json_sha256=sha(WS/'docs/evidence/W13/formal-v1'/(r['run_id']+'.json')),
                reason='Expected missing initialpose/AMCL TF on an intact observed startup failure; arrival remains false'))
            row['outcome']=derived
            row['notes']='raw worker '+original+'; offline '+derived+'; startup failed before a goal was sent'
    analysis=dict(code_sha=subprocess.check_output(['git','-C',str(WS),'rev-parse','HEAD'],text=True).strip(),
        corrections=corrections,success_count_unchanged=True)
    (out/'classification.json').write_text(json.dumps(analysis,indent=2)+'\n')
    perf={'formal_headless_capped':performance('formal-v1',runs)}
    for campaign in ('performance-headless-v1','performance-gui-v1'):
        if (WS/'docs/evidence/W13'/campaign/'summary.json').exists():
            _,_,rr=load_campaign(campaign);perf[campaign]=performance(campaign,rr)
    (out/'performance.json').write_text(json.dumps(perf,indent=2)+'\n')
    failures=[r for r in summary['rows'] if r['outcome']!='SUCCESS']
    clearances=[r['body_clearance']['min_m'] for r in runs.values() if 'body_clearance' in r]
    stops=[event for r in runs.values() for event in protective_braking(r)]
    (out/'protective-braking.json').write_text(json.dumps(dict(definition='CM stop/slowdown edge associated with zero policy within .5s; includes slowdown to gait deadzone; max displacement until next nonzero policy, not a controlled brake experiment',events=stops),indent=2)+'\n')
    stopd=[s['max_displacement_after_zero_m'] for s in stops if 'max_displacement_after_zero_m' in s]
    detours=[r['case_spec']['id'] for r in runs.values() if r['case_spec']['suite']=='blocking' and r['outcome']=='SUCCESS' and r.get('dynamic',{}).get('checks',{}).get('real_detour')]
    checks=dict(forty_rows=len(summary['rows'])==40,all_run_sources=len(runs)==40,no_not_run=summary['complete'],
        only_document_dirty=all(r.get('validity_checks',{}).get('source_clean') for r in runs.values()),
        validity=all(all(r.get('validity_checks',{}).values()) or (not r.get('nav_active_sim') and classify(r,False) in ('FAIL_STARTUP','FAIL_LOCALIZATION')) for r in runs.values()),
        static_10_of_10=summary['counts']['static']['success']==10,
        dynamic_9_of_10=all(summary['counts'][s]['success']>=9 for s in ['crossing','blocking','sudden']),
        zero_contacts=all(r.get('metrics',{}).get('env_contacts')==dict(static_env=0,obstacle=0,ground_nonfoot=0) for r in runs.values()),
        zero_falls=all(r.get('metrics',{}).get('fell') is False and r['manifest']['status']!='FALLEN' for r in runs.values()),
        successful_blocking_detour=bool(detours),processes_reaped=all(not r.get('run',{}).get('remaining_owned_pids') for r in runs.values()),
        unique_ids=len({r['run_id'] for r in runs.values()})==40,
        common_parameter_hashes=all(len({r['manifest']['parameter_files'][key]['sha256'] for r in runs.values()})==1 for key in ['nav2','command_map','collision_monitor','velocity_smoother']))
    for field in ('settle_analysis','clearances'):
        a=ast.parse((WS/'docs/evidence/W12/nav_run.py').read_text());b=ast.parse((WS/'scripts/evaluation/episode.py').read_text())
        checks['frozen_'+field]=ast.dump(next(n for n in a.body if isinstance(n,ast.FunctionDef) and n.name==field))==ast.dump(next(n for n in b.body if isinstance(n,ast.FunctionDef) and n.name==field))
    checks['behavior_files_unchanged']=subprocess.check_output(['git','-C',str(WS),'diff','e31e18313d05b9023c18b4d43e342409572d7c23','--','src/t1_navigation','src/t1_mujoco/worlds','src/t1_mujoco/config','src/t1_bringup/t1_bringup','src/t1_bringup/launch'],text=True)==''
    runtime_shas=sorted({r['manifest']['code_sha'] for r in runs.values()})
    checks['formal_sources_identical']=all(subprocess.check_output(['git','-C',str(WS),'diff',state['registration_sha'],sha_value,'--','src','scripts'],text=True)=='' for sha_value in runtime_shas)
    checks['one_checkpoint']=len({r['manifest']['checkpoint_sha256'] for r in runs.values()})==1
    checks['sensor_odom_declared']=all(r['manifest']['sensor_mode']=='compensated' and r['manifest']['odom_mode']=='ideal_truth' for r in runs.values())
    class RemoveTimers(ast.NodeTransformer):
        def visit_Assign(self,node):
            names=[t.attr if isinstance(t,ast.Attribute) else t.id if isinstance(t,ast.Name) else '' for t in node.targets]
            return None if all(n in {'policy_began','physics_began','policy_compute_ms','physics_compute_ms'} for n in names) else self.generic_visit(node)
    previous=ast.parse(subprocess.check_output(['git','-C',str(WS),'show','e31e18313d05b9023c18b4d43e342409572d7c23:src/t1_mujoco/t1_mujoco/runner.py'],text=True))
    current=RemoveTimers().visit(ast.parse((WS/'src/t1_mujoco/t1_mujoco/runner.py').read_text()))
    checks['runner_control_ast_unchanged']=ast.dump(previous)==ast.dump(current)
    for campaign,threshold in [('performance-headless-v1',1.),('performance-gui-v1',.7)]:
        if campaign in perf:
            checks[campaign+'_rtf']=perf[campaign]['samples']['rtf'].get('median',0)>=threshold
    if 'performance-gui-v1' in perf:
        checks['gui_render_artifacts']=len(list((WS/'docs/evidence/W13/performance-gui-v1').glob('*-final*.png')))>=2
    audit=dict(checks=checks,verdict='PASS' if all(checks.values()) else 'FAIL',counts=summary['counts'],body_min_m=min(clearances) if clearances else None,
               stopping_displacement_m=distribution(stopd),successful_blocking_cases=detours,formal_registration_sha=state['registration_sha'],source_fingerprint=state['fingerprint'],runtime_shas=runtime_shas,analysis_code_sha=analysis['code_sha'])
    (out/'audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    fields=next(csv.reader((WS/'docs/evidence/W13/formal-v1/results.csv').open()))
    for filename,rows in [('results.csv',summary['rows']),('failures.csv',failures)]:
        with (out/filename).open('w') as f:
            writer=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');writer.writeheader();writer.writerows(rows)
    import re
    failure_evidence=[]
    for row in failures:
        r=runs[row['case']];log=WS/'docs/evidence/W13/raw/formal-v1'/r['run_id']/'raw'/(r['run_id']+'.txt')
        lines=log.read_text(errors='replace').splitlines()
        evidence=[dict(line=i+1,text=line) for i,line in enumerate(lines) if re.search(r'ERROR|CRITICAL|timed out|failed to send response|Aborting handle|Failed to make progress|Failed to create plan|No valid path',line,re.I)]
        failure_evidence.append(dict(case=row['case'],outcome=row['outcome'],code_sha=r['manifest']['code_sha'],
            run_id=r['run_id'],log_path=str(log.relative_to(WS)),log_sha256=sha(log),excerpts=evidence[:50]))
    (out/'failure-evidence.json').write_text(json.dumps(failure_evidence,indent=2)+'\n')
    text=['# W13 固定40例导航评测','',f'状态：**{summary["verdict"]}**。完整40例；未缩减样本。只执行W13，W14未启动。',
      '',f'正式注册/被测起始提交：`{state["registration_sha"]}`；全套source/config内容指纹`{state["fingerprint"]}`。各run完整SHA、实际dirty、参数/地图/策略哈希见[结果来源](../evidence/W13/formal-v1/summary.json)及逐run JSON。',
      '', '| 类别 | 实际到达/总数 | 门槛 | 结果 |','|---|---:|---:|---|']
    for label,count in summary['counts'].items():
        threshold=10 if label=='static' else 9
        text.append(f'| {label} | {count["success"]}/{count["n"]} | ≥{threshold}/10 | {"PASS" if count["success"]>=threshold else "FAIL"} |')
    text += ['', '每例均完整重启MuJoCo/策略、ROS节点/TF缓存、AMCL和costmap；种子、起终点及障碍速度/触发/停留时间在正式运行前登记于test_suite_v1.yaml和w13_events_v1.yaml。S01–S05保留W11五条路线，S06–S10重复路线但采用新种子；没有删行或失败后换路线。MPPI/ROS调度存在非确定性，种子不保证逐位重放。',
      '', '判据维持≤.30m、≤20°、0.5s窗线/角速<.05、连续稳定2s，任务上限120s仿真时间。需要Nav2成功、定位工具核验和独立真值核验同时通过。超时、安全取消或仅action成功均不计实际到达。',
      '',f'环境/障碍接触零：{checks["zero_contacts"]}；跌倒零：{checks["zero_falls"]}。成功长期阻塞绕行：{", ".join(detours) or "无"}。具体证据和全部失败保留于[results.csv](results.csv)、[failures.csv](failures.csv)，开发pilot-v1/v2及性能专用运行分别保存，不混入分母。',
      '', f'全身碰撞几何对非地面环境/活动障碍的三维有符号净距最小采样值：{min(clearances) if clearances else None:.6f}m。每10Hz记录qpos与mocap，run退出后在独立模型重放mj_geomDistance；排除地面、自碰撞、visual-only。此值为采样最小值，不是连续最小距离证明。500Hz原有接触判定补充无碰撞检查。footprint为W07保守多边形的10Hz边界采样距离（约2cm采样间隔，含动态体），LiDAR最小量程另列；三者不能混称。',
      '',f'安全停止沿用W10 1.65/1.95m圆，命令源整形→gate→smoother→CM→只减不增bridge。停车事件按任务接受后CM stop上升沿/bridge故障零命令上升沿去重。去除启动停车，CM stop或slowdown后相关零策略事件数{len(stops)}，首次零策略目标后至恢复非零目标/记录结束的最大平面位移统计：{json.dumps(distribution(stopd),ensure_ascii=False)}。逐事件含触发时刻、速度、置零延迟、W05静止窗口时刻及恢复时刻；短暂等待不足2s的事件静止时刻为null，不能声称通过独立制动试验。',
      '', '碰撞沿用W11/W12每2ms检测：机器人root=trunk碰撞几何对非地面环境/障碍；脚底地面接触排除，非足地面接触记失败。连续geom对一个事件，断开≥.04s才新建事件。每run记录三类计数及contact_events。',
      '', '版本例外ADR-018经用户明确授权：仅用户docs/start_commands.md可dirty，每例保留dirty=true、文件列表和开始/结束哈希。用户文档未暂存/覆盖；代码、参数和测试集必须已提交，其他dirty仍INVALID。raw结果在ignored目录生成，整批运行后导出，避免结果污染下一例。',
      '', '失败定位：详见下表。终点过冲/原地偏航回弹、未恢复的导航失败归W11 ADR-016及W05步态包络的集成；动态体真实感知与重规划问题归W12。启动/定位失败分别依据navigation active、initialpose、图/TF和launch日志分类。W13不修改控制器、安全区、地图或物理dt来提高统计。历史W11/W12冒烟证据仍对应各自当时样本，不能代表本批可靠率。',
      '', '| case | outcome | action | truth位置/yaw | 诊断 |','|---|---|---|---|---|']
    for row in failures:
        r=runs.get(row['case'],{});m=r.get('metrics',{})
        diagnosis=f"终点重试{m.get('terminal_retries')}；progress_failure={m.get('log_counts',{}).get('progress_failure')}；action错误{m.get('nav2_error_code')}"
        if row['case']=='C01': diagnosis='AMCL/lifecycle启动未完成，未发送目标（W11启动/定位集成）'
        elif row['case']=='C09': diagnosis='follow_path ACK与costmap服务超时（W11 BT/就绪）'
        elif row['case']=='B04': diagnosis='绕行后终点过冲，NO_VALID_PATH208（W11末端进近）'
        elif row['case']=='B06': diagnosis='绕行后终点未收敛，120s取消（W11末端进近）'
        text.append(f'| {row["case"]} | {row["outcome"]} | {row.get("nav2_result")} | {row.get("pos_err_m")}m / {row.get("yaw_err_deg")}° | {diagnosis} |')
    text += ['', '限制：理想真值odom、真值姿态补偿、有限垂直束扫描、平地/宽通道；mocap实体不可被推开，反应式避障无目标跟踪预测。40次同机仿真样本不代表真实世界可靠性。没有进行可选安全链关闭/里程计漂移消融，优先完整主测试集。', '', '性能见[performance.md](performance.md)，审计逐项见[audit.json](audit.json)。未满足原目标则W13为FAILED，不进入W14。', '', 'C01原worker把缺失initialpose/TF标INVALID；[classification.json](classification.json)统一离线修正为FAIL_STARTUP，原始JSON/CSV保留，成功数未变。失败日志摘录见[failure-evidence.json](failure-evidence.json)。', '', '![全部40条物理轨迹](trajectories.png)']
    (out/'evaluation.md').write_text('\n'.join(text)+'\n')
    hardware=json.loads((WS/'docs/evidence/W13/start-state.json').read_text())
    lines=['# W13 性能测量','', '硬件：Intel Core Ultra 7 155H，WSL2 Ubuntu 24.04，22逻辑CPU；ROS2 Jazzy、MuJoCo3.15.0，具体Python/Torch/解释器、线程、checkpoint和命令行见逐run manifest。torch.set_num_threads(1)，同一进程owner推进2ms物理/20ms策略，9×360束扫描10Hz，Nav2 controller20Hz/MPPI1000×56，visualize=false、无ghost。',
        '', '正式40例采用target_rtf=1节拍。独立性能campaign使用headless target_rtf=1.1、GUI target_rtf=1.0，仅测该节拍是否可达，不声称极限容量；不混入正式可靠率。每模式只有一例S04，分布样本为该运行内策略/扫描/CPU窗口。GUI含实际MuJoCo passive viewer 30Hz和RViz；截图随专用campaign保存。',
        '', '| 运行组 | 指标 | N | 中位数 | P5 | P95 | 最大 |','|---|---|---:|---:|---:|---:|---:|']
    for campaign,data in perf.items():
        for key,d in data['samples'].items():
            lines.append(f'| {campaign} | {key} {"" if key=="rtf" else "ms"} | {d["n"]} | {d.get("median",0):.5f} | {d.get("p5",0):.5f} | {d.get("p95",0):.5f} | {d.get("max",0):.5f} |')
        for key,d in data['navigation_cpu_ms_per_wall_s'].items():
            lines.append(f'| {campaign} | {key} CPU ms/wall-s | {d["n"]} | {d.get("median",0):.3f} | {d.get("p5",0):.3f} | {d.get("p95",0):.3f} | {d.get("max",0):.3f} |')
    lines += ['', 'RTF来自10s墙钟滑窗Δsim/Δwall，排除前10s墙钟启动段；其他耗时为所有策略步的墙钟计时，scan仅统计due样本。physics_compute_ms包含10个PD/物理子步、mocap及接触钩子；policy_compute_ms只包policy_step；step_compute_ms还含状态处理。',
        '', '导航耗时为Linux /proc 对自持进程约0.5s墙钟窗口采样的user+sys CPU增量，归一化为CPU ms/wall-s；controller_server包含costmap和控制线程，不能解释为单次控制回调延迟。仅统计首个目标接受至工具结果结束之间的活动任务样本。按进程区分planner/controller/BT/behavior/AMCL/CM，完整采样及原trace留ignored raw并保存SHA。',
        '', '优化选择：现有配置已关闭MPPI采样轨迹visualize与ghost，raycast已批处理。本步增加观测并测量，未改变策略、物理dt/decimation、安全参数、扫描分辨率或Nav2参数。没有为改善数字改变/clock；若GUI未达0.7或headless未达1.0如实标记。']
    for name,threshold in [('performance-headless-v1',1.),('performance-gui-v1',.7)]:
        if name in perf:
            median=perf[name]['samples']['rtf']['median'];lines.append(f'\n{name}: 中位数{median:.6f}，目标≥{threshold}：{"PASS" if median>=threshold else "FAIL"}。')
    (out/'performance.md').write_text('\n'.join(lines)+'\n')
    fig,axes=plt.subplots(1,4,figsize=(16,4))
    for ax,suite in zip(axes,['static','crossing','blocking','sudden']):
        for result in runs.values():
            if result['case_spec']['suite']!=suite: continue
            path=WS/'run_data'/result['run_id']/'sim_trace.csv';_,tr=trace_columns(path)
            ax.plot(tr['x_w'],tr['y_w'],lw=.8,label=result['case_spec']['id']+(' OK' if result['outcome']=='SUCCESS' else ' FAIL'))
        ax.set(title=suite,xlabel='world x (m)',ylabel='world y (m)',xlim=(-2,11.5),ylim=(-5.5,5.5));ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=5,ncol=2)
    fig.tight_layout();fig.savefig(out/'trajectories.png',dpi=160);plt.close(fig)
    print(json.dumps(audit,indent=2))


if __name__=='__main__': main()
