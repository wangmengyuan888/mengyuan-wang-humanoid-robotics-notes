# 环境配置教程

[返回仓库总目录](../README.md)

集中收录个人环境笔记，按版本分开阅读。保留历史操作和排错，不将安装记录当成所有仿真、训练或真机已经验证。

## 文件说明

| 文件 | 内容与阅读用途 |
| --- | --- |
| [00_Docker Ubuntu20.04 ROS1 版本为 Noetic，ROS2 版本为Foxy环境配置.md](<00_Docker Ubuntu20.04 ROS1 版本为 Noetic，ROS2 版本为Foxy环境配置.md>) | 作业使用的 Docker、Ubuntu 20.04、Noetic/Foxy、GPU/显示/代理和 ROS 1/2 桥接；[作业参考笔记目录](../02_作业参考笔记/00_环境配置与ROS桥接.md)也保留一份，文件名为 `00_环境配置与ROS桥接.md`。 |
| [01_Ubuntu22.04安装与扩容.md](01_Ubuntu22.04安装与扩容.md) | 启动盘、安装与黑屏处理、扩容入口及缓存维护；缺少完整分区操作证据。 |
| [02_ROS2Humble安装与Isaac桥接.md](02_ROS2Humble安装与Isaac桥接.md) | Humble 安装、通信测试、工作空间、Isaac Bridge、/clock 与仿真时间。 |
| [03_MuJoCo与ros2_control.md](03_MuJoCo与ros2_control.md) | MuJoCo 模型、MUJOCO_DIR、ros2_control 工程和 GLFW 构建排错。 |
| [04_IsaacSim5.1与IsaacLab2.3实践.md](04_IsaacSim5.1与IsaacLab2.3实践.md) | 本地 Ubuntu 中 Sim 5.1.0/Lab 2.3.0 配置安装、Python ABI、外部任务与 PPO/skrl；另保留 AutoDL 尝试。 |
| [05_IsaacSim6.1与IsaacLab3.0实践.md](05_IsaacSim6.1与IsaacLab3.0实践.md) | release 分支、Python/Kit、安装选项、Humble 桥接与视角操作。 |

## 阅读方式

按编号顺序阅读；已有基础时可直接进入对应专题。正文中的来源页码、代码路径、版本条件及缺失说明用于核对知识与复现条件。

根据自己的系统版本选择教程，00 是 Ubuntu 20.04/Noetic/Foxy 的历史环境，02、04、05 涉及 Humble；Sim 5.1/Lab 2.3 与 Sim 6.1/Lab 3.0 的 Python、依赖和安装选项分别说明。[环境截图说明](assets/README.md)。
