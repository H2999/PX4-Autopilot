# MicoAir743v2-AIO-35A RMUC 2027 装机与固件使用说明

## 1. 固件编译与烧录目标

`MicoAir743v2-AIO-35A` 必须使用 **`micoair_h743-v2`** 目标（**严禁使用 `micoair_h743-aio`**）：

```bash
make micoair_h743-v2_default
```

生成固件路径：`build/micoair_h743-v2_default/micoair_h743-v2_default.px4`（可通过 QGC 或 MicoConfigurator 烧录）。

## 2. 串口与供电接线清单

| AIO 物理接口 | PX4 端口 | 连接对象 | 关键注意事项 |
| :--- | :--- | :--- | :--- |
| `UART1` (`PA9/PA10`) | `TELEM1` | Orange Pi Zero 3W | `921600` bps，MAVSDK Offboard |
| `UART4` (`PA0/PA1`) | `TELEM2` | 黑森矩阵 RoboBaton Neo | `921600` bps，MAVLink 2 `ODOMETRY`(ID 331)；建议 6pin 接 `TX/RX/GND` + AIO 5V 备用 `VIN` |
| `UART6` (`PC6/PC7`) | `RC` | S79 急停遥控接收机 | 必须验证 CH8 急停与关控自动停桨 |
| `UART2` (SH1.0-6P) | `GPS2` | **悬空禁用** | 第 1 脚为 **12V**，禁止接入 5V 设备 |
| `VBAT / GND` | 主电源 | 2S LiPo (≤ 4Wh) + 外部 5V/3A BEC | AIO 板载 5V BEC 上限仅 2A，**Orange Pi Zero 3W（及 USB 上的 Neo）必须由外部独立 5V/3A BEC + 470–1000µF 电容供电** |

## 3. 导入参数与拆桨地面验收

1. **拆桨**状态下，通过 QGC 参数页面加载 `rm27_aio35a_neo_impact.params` 并重启；
2. 核对基础项：
   - 2S 电池显示正常电压（~7.4–8.4V，`BAT1_N_CELLS=2`、`BAT1_V_DIV=21.12`、`BAT1_A_PER_V=14.14`）；
   - 45° 菱形安装时确认板上箭头正指机头，QGC HUD 滚转/俯仰方向完全正确；
   - Bluejay 电调为 `DShot300`（`PWM_MAIN_TIM0=-4`），电机编号与转向核对无误；
   - 关闭急停遥控器确认自动停桨（规则 S79）；
3. 核对 RoboBaton Neo 定位：
   - 访问 `http://192.168.8.1` 配置安装外参（默认水平前装）；
   - 在 MAVLink Inspector / NSH `listener vehicle_visual_odometry` 中确认 30 Hz 里程计正常流入，`vehicle_local_position` 随手持移动方向与尺度正确；
4. 核对 1 kHz 撞击反射（`ImpactReflex`）：
   - 默认 `MC_IMP_MODE=0`（完全禁用，不影响常规飞行）；
   - 上位机 MAVSDK 在末端 4–6 m/s 冲刺前设置 `MC_IMP_MODE=1`（60° 基地正斜面）或 `MC_IMP_MODE=2`（105°/120° 前哨站倒悬面）；
   - 撞击反射为**单次触发锁存（One-Shot）**：触发完成 `DetachDrop → CatchRecover → Done` 后自动把控制权交还 Offboard，下次复撞前由上位机先置 `0` 再置 `1/2` 重新武装；
   - 任何时刻切出 Offboard 或触发急停，反射立即无条件退出。
