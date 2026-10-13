"""W02 C4 mechanical consistency check of docs/architecture/*.md.

Checks what can be checked by text: required interfaces/sections present,
TF child uniqueness, topic table completeness, facts consistent with the C1
probe, ADR references exist, relative links resolve. Human review is still
required for semantics. Usage (from $WS):
    python3 docs/evidence/W02/check_contracts.py
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[3]
ARCH = WS / "docs/architecture"
docs = {n: (ARCH / n).read_text(encoding="utf-8") for n in ("system.md", "interfaces.md", "acceptance.md")}
adr = (WS / "docs/plans/t1_mujoco_navigation/决策记录.md").read_text(encoding="utf-8")
probe = json.loads((WS / "docs/evidence/W02/c1-probe.json").read_text(encoding="utf-8"))
results: list[tuple[bool, str]] = []


def check(ok: bool, msg: str) -> None:
    results.append((bool(ok), msg))


def section(text: str, heading_prefix: str) -> str:
    m = re.search(rf"^## {re.escape(heading_prefix)}.*?$(.*?)(?=^## |\Z)", text, re.S | re.M)
    return m.group(1) if m else ""


def table_rows(text: str) -> list[list[str]]:
    rows = []
    for line in text.splitlines():
        if line.startswith("|") and not re.match(r"^\|\s*-{3}", line):
            rows.append([c.strip() for c in line.strip().strip("|").split("|")])
    return rows[1:] if rows else []


I = docs["interfaces.md"]
S = docs["system.md"]
A = docs["acceptance.md"]

# C2 required interfaces
for t in ["/clock", "/odom", "/joint_states", "/scan", "/imu/data", "/cmd_vel_nav", "/cmd_vel_smoothed",
          "`/cmd_vel`", "/diagnostics", "/t1/sim_state", "/t1/eval/ground_truth"]:
    check(t in section(I, "2. 话题"), f"interfaces §2 lists {t}")
topic_rows = table_rows(section(I, "2. 话题"))
check(len(topic_rows) >= 15, f"topic table rows={len(topic_rows)} >= 15")
for r in topic_rows:
    check(len(r) == 9 and all(r), f"topic row complete (9 non-empty cols): {r[0]}")
for kw in ["enable_stamped_cmd_vel: false", "use_sim_time: true", "data.time", "单调", "use_sim_time=false"]:
    check(kw in I, f"interfaces mentions '{kw}'")

# TF uniqueness
tf_rows = table_rows(section(I, "4. Frame"))
children = [r[0].strip("`") for r in tf_rows]
check(len(children) == len(set(children)), f"TF children unique: {children}")
check(all(r[2] for r in tf_rows), "every TF child has a named publisher")
check("XOR" in section(I, "4. Frame") or "互斥" in section(I, "4. Frame"), "map→odom exclusivity stated")

# C3 command semantics
for kw in ["v_des", "u_pol", "v_meas", "M(0)=0", "只减不增", "禁止**以全关节力矩置零", "command_map.yaml", "待W05确定"]:
    check(kw in I, f"interfaces §6 contains '{kw}'")

# reset/pause table
r73 = table_rows(section(I, "7. 状态机"))
check(any("暂停" in r[0] for r in r73) and any("原地复位" in r[0] for r in r73) and any("回跳" in r[0] for r in r73),
      "pause/reset/time-jump rows present")

# facts vs probe
task = probe["task"]
check(task["policy_dt"] == 0.02 and "policy_dt=0.02" in I, "policy_dt 0.02 consistent")
check(task["decimation"] == 10 and "decimation=10" in I, "decimation 10 consistent")
m = probe["model"]
check(m["nq"] == 30 and m["nv"] == 29 and m["nu"] == 23 and "nq=30、nv=29、nu=23" in I, "nq/nv/nu consistent")
check(probe["checks"]["qvel[0:3] is WORLD-frame linear vel (local lin ~ [0,-1,0])"] and "**世界系**线速度" in I,
      "qvel[0:3] world-frame fact consistent")
check(probe["checks"]["gyro sensor == qvel[3:6] (noise attr not applied)"] and "无自动噪声" in I, "IMU truth fact consistent")
check(probe["all_pass"], "C1 probe all_pass")

# C4 architecture items
for kw in ["直接修改上游", "共同写 MjData", "纯真值地图导航"]:
    check(kw in section(S, "11."), f"system §11 alternative '{kw}'")
for w in [f"W{i:02d}" for i in range(3, 13)]:
    check(re.search(rf"^\| {w} \|", section(S, "9."), re.M) is not None, f"system §9 row {w}")
for kw in ["/clock 双发布", "TF 环路", "重复状态真相", "隐含依赖"]:
    check(kw in section(S, "10."), f"system §10 review '{kw}'")
check("```text" in section(S, "6.") and len(table_rows(section(S, "5."))) >= 10, "data-flow diagram and control-cycle table")
for kw in ["run_id", "manifest.json", "sim_trace.csv", "results.csv", "fault_tests.csv", "1000–1009", "F13"]:
    check(kw in A, f"acceptance contains '{kw}'")

# ADR references exist
for ref in sorted(set(re.findall(r"ADR-\d{3}", adr))):
    check(f"| {ref} |" in adr, f"{ref} defined")
for n in range(1, 9):
    check(f"| ADR-{n:03d} |" in adr, f"ADR-{n:03d} present")

# relative links resolve
for name, text in docs.items():
    for link in re.findall(r"\]\(([^)#]+)(?:#[^)]*)?\)", text):
        if "://" in link:
            continue
        check((ARCH / link).resolve().exists(), f"{name} link -> {link}")

fails = [msg for ok, msg in results if not ok]
for ok, msg in results:
    print(("PASS " if ok else "FAIL ") + msg)
print(f"TOTAL {len(results)} FAIL {len(fails)}")
sys.exit(1 if fails else 0)
