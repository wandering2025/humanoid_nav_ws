# W14 RViz生命周期严格复验：最终BLOCKED

起始f5f401165270750b777630ad80c3010dd816462b、clean。离线假设为context/display销毁竞争，原具体堆破坏对象未知。
新薄宿主正常析构、NVIDIA明确绑定，不改系统库或导航参数。34+131非图形测试、编译、静态检查通过后才运行唯一最小。

- minimal-result.json：20261015T134828Z_rviz_minimal_6d4beca0，PASS，3进程均0；NVIDIA4050，51°C/851MiB，37.7332s。
- full-01-final-result.json：20261015T135052Z_demo_U01_825a6e42-ad9d-45b3-a76c-26e30466af89，SUCCESS，全组件0。
- full-02-final-result.json：20261015T135427Z_demo_U01_32c2cee4-f75e-4bd9-8e91-faccdb1d6217，FAILED，DDS−11；RViz与其余组件0。

两轮同一干净a963518410ad0faefad0c3b2230d367780c91212，同命令./scripts/run_demo.sh、同U01/4000，导航/dynamic均PASS，碰撞/跌倒0。
安全停车、无PID和端口释放有独立审计；没有SIGKILL。首轮审计后才第二轮；第二轮后停止，没有第三轮或另一条路径。
Windows/WSLg新增驱动错误未匹配；Linux内核dxg查询−22/−2有记录，未证明无害。DDS线程在libfastrtps内SIGSEGV由内核直接确认。
原始JSON、负退出码与全部raw保留；strict-raw-index.json是文件hash清单。先前W13/黑屏/CPU/−6失败不改。
最终实现175752b6e14fafbbdd2f21395f466ea5f099b734。录像MISSING，W15未启动，W14_BLOCKED。
