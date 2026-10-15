> 当前优化版本仍有GPU告警启动锁，尚未授权/完成图形恢复。下述独立录像流程仅在恢复验证通过后执行；旧发布02录像不代表本轮优化。

# 独立录像流程

正常演示入口不启动录像。发布视频见[清单](release-manifest.md)。以下只用于明确需要另录一轮时，输出目录必须是新的、Git ignored的目录，源码必须干净。

先运行推荐演示命令。在新run目录出现后、机器人开始运动前，从高级worktree的另一终端启动独立读取：

```bash
/usr/bin/python3 scripts/advanced/record.py capture --run-dir /完整路径/run_data/advanced/本次RUN_ID --output /完整路径/docs/evidence/advanced/raw/新的录制目录
```

读取器等待两张完整ready截图，确认唯一MuJoCo/RViz窗口后顺序抓取，最多4组/秒；它不发布ROS命令、不控制机器人、不创建渲染器，也不在运行时编码。到达且确认停稳后再保留5秒，随即停止。若抓取失败，本次演示仍按自己的安全生命周期结束。

只有演示显示Final SUCCESS并完成端口释放后，才离线CPU编码。以下命令以本次验证使用的设备访问限制运行编码器：

```bash
/usr/bin/python3 -c 'import sys,runpy;sys.path.insert(0,"scripts/demo");from cpu_guard import enter;enter();sys.argv=["record.py",*sys.argv[1:]];runpy.run_path("scripts/advanced/record.py",run_name="__main__")' encode --run-dir /完整路径/run_data/advanced/本次RUN_ID --output /完整路径/docs/evidence/advanced/raw/新的录制目录
```

编码器核对来源SHA、SUCCESS、动态判据、四项零接触/跌倒、停稳和清理；保持真实单调时钟时间，用最近的已抓取帧补齐4fps时间轴；尾部8秒明确标记运行后证据卡。它完整软件解码生成文件并记录哈希、时长、尺寸、帧数、采样间隔与两窗口时间差。

完整解码不能代替视觉验收。还须检查A、初始地图、连续分区建图、从当前位置接续导航、实际规划路径、障碍接近/停留/撤离、继续到B、停稳与证据卡；visual_review_pending默认为true。未通过的录制原帧和日志不得删除或冒充成功录像。发布02的审查过程和脚本均在其raw目录及Git哈希索引中。
