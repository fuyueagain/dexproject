# XLeRobot 舵机控制范围报告

> 初次检测: 2026-03-28
> 校准完成: 2026-03-28 (左臂 + 右臂)
> 舵机型号: 飞特 STS3215 (4096 步/圈, 0-4095 = 0°-360°)
> 数据来源: `lerobot_measure_feetech_ranges` 自动校准 + 硬件寄存器读取

---

## 概览

| 项目 | 状态 |
|------|------|
| 总线数量 | 4 条 (ttyACM0 ~ ttyACM3) |
| 舵机总数 | 29 个 |
| 左臂从臂校准 | **已完成** — 硬件限位已写入 EEPROM |
| 右臂从臂校准 | **已完成** — 硬件限位已写入 EEPROM |
| 主臂 (ACM2/3) | 未校准 (遥操用, 无执行器控制需求) |
| 头部云台 / 底盘 | 未校准 (单独控制逻辑) |

---

## 1. 校准结果：左臂从臂 (/dev/ttyACM0, ID 1-6)

校准文件: `~/.cache/huggingface/lerobot/calibration/robots/so_follower/arm_left.json`
完整日志: `dexproject/calibration_left_arm.log`

| 关节 | ID | 标准名 | 功能 | range_min | range_max | 可用范围 (步/°) | homing_offset | 堵转位置 (CW/CCW) |
|:----:|:--:|:------:|:----:|:---------:|:---------:|:--------------:|:-------------:|:-----------------:|
| 1 | 1 | shoulder_pan | 底座左右旋转 | 658 | 3436 | 2778 / 244.2° | 1863 | 1203 / 2521 |
| 2 | 2 | shoulder_lift | 肩部上下俯仰 | 844 | 3250 | 2406 / 211.5° | -1370 | 1880 / 3570 |
| 3 | 3 | elbow_flex | 肘部上下俯仰 | 918 | 3176 | 2258 / 198.5° | 1654 | 734 / 2572 |
| 4 | 4 | wrist_flex | 腕部上下俯仰 | 828 | 3266 | 2438 / 214.3° | -1322 | 1944 / 3602 |
| 5 | 5 | wrist_roll | 腕部左右旋转 | 497 | 3597 | 3100 / 272.5° | 1050 | 551 / 1547 |
| 6 | 6 | gripper | 夹爪开合 | 1250 | 2844 | 1594 / 140.1° | -1935 | 910 / 3411 |

---

## 2. 校准结果：右臂从臂 (/dev/ttyACM1, ID 1-6)

校准文件: `~/.cache/huggingface/lerobot/calibration/robots/so_follower/arm_right.json`
完整日志: `dexproject/calibration_right_arm.log`

| 关节 | ID | 标准名 | 功能 | range_min | range_max | 可用范围 (步/°) | homing_offset | 堵转位置 (CW/CCW) |
|:----:|:--:|:------:|:----:|:---------:|:---------:|:--------------:|:-------------:|:-----------------:|
| 1 | 1 | shoulder_pan | 底座左右旋转 | 655 | 3439 | 2784 / 244.7° | -1835 | 1604 / 2916 |
| 2 | 2 | shoulder_lift | 肩部上下俯仰 | 869 | 3225 | 2356 / 207.1° | -1626 | 1599 / 3339 |
| 3 | 3 | elbow_flex | 肘部上下俯仰 | 915 | 3179 | 2264 / 199.0° | -1099 | 2080 / 3912 |
| 4 | 4 | wrist_flex | 腕部上下俯仰 | 844 | 3250 | 2406 / 211.5° | -1461 | 1790 / 3479 |
| 5 | 5 | wrist_roll | 腕部左右旋转 | 108 | 3986 | 3878 / 340.9° | 1263 | 1154 / 1371 |
| 6 | 6 | gripper | 夹爪开合 | 1249 | 2845 | 1596 / 140.3° | -1697 | 1149 / 3648 |

---

## 3. 左右臂对比

| 关节 | 左臂可用范围 | 右臂可用范围 | 差异 |
|:----:|:----------:|:----------:|:----:|
| shoulder_pan | 2778 步 (244.2°) | 2784 步 (244.7°) | 一致 |
| shoulder_lift | 2406 步 (211.5°) | 2356 步 (207.1°) | 4.4° |
| elbow_flex | 2258 步 (198.5°) | 2264 步 (199.0°) | 一致 |
| wrist_flex | 2438 步 (214.3°) | 2406 步 (211.5°) | 2.8° |
| wrist_roll | 3100 步 (272.5°) | 3878 步 (340.9°) | **68.4°** |
| gripper | 1594 步 (140.1°) | 1596 步 (140.3°) | 一致 |

> wrist_roll 差异较大 (68°)，可能是右臂腕部旋转关节的物理止挡间距更大，或安装位置不同。

---

## 4. /dev/ttyACM0 — 底盘三轮 (ID 7-9)

底盘三轮使用速度模式, 无需位置校准。限位仍为出厂默认 0-4095。

| 轮 | ID | 标准名 | 功能 | Torque_Limit | 加速度 |
|:--:|:--:|:------:|:----:|:------------:|:------:|
| 1 | 7 | base_left_wheel | 底盘左轮 | 1000 | **254** |
| 2 | 8 | base_back_wheel | 底盘后轮 | 1000 | **254** |
| 3 | 9 | base_right_wheel | 底盘右轮 | 1000 | 0 |

> ID9 加速度为 0 与 ID7/8 (254) 不一致

---

## 5. /dev/ttyACM1 — 头部云台 (ID 7-8)

未校准, 限位为出厂默认 0-4095。

| 轴 | ID | 标准名 | 功能 | Torque_Limit |
|:--:|:--:|:------:|:----:|:------------:|
| 水平 | 7 | head_pan | 云台左右旋转 | 1000 |
| 垂直 | 8 | head_tilt | 云台上下俯仰 | 1000 |

---

## 6. 校准文件清单

校准数据路径: `~/.cache/huggingface/lerobot/calibration/`

| 文件 | 内容 | 状态 |
|------|------|------|
| `robots/so_follower/arm_left.json` | 左臂从臂 6 关节校准 | **已校准** (2026-03-28) |
| `robots/so_follower/arm_right.json` | 右臂从臂 6 关节校准 | **已校准** (2026-03-28) |
| `robots/xlerobot/test_xlerobot.json` | 底盘三轮 | 默认值 (速度模式无需校准) |

完整校准日志:
- `dexproject/calibration_left_arm.log`
- `dexproject/calibration_right_arm.log`

---

## 7. 原仓库防自碰撞机制分析

### 7.1 原仓库提供的安全机制

经过对 `lerobot-MakerMods` 仓库的完整检查，**原仓库没有内置运动学级别的自碰撞检测**。提供了以下三层防护，但均需要正确配置后才能生效：

#### 第一层：硬件位置限位 (Min_Position_Limit / Max_Position_Limit)

- 存储在 STS3215 舵机 EEPROM 中
- 校准脚本 `lerobot_measure_feetech_ranges` 会探测每个关节的实际机械极限，并写入这两个寄存器
- **当前状态：全部为 0-4095 (出厂默认)，未生效**

#### 第二层：归一化裁剪 (range_min / range_max)

- 代码位置：`motors_bus.py` → `_normalize()` / `_unnormalize()`
- 在读取/写入位置时，会将原始值裁剪到 `[range_min, range_max]` 范围内：

```python
# motors_bus.py 第 783 行
bounded_val = min(max_, max(min_, val))
```

- 校准文件中的 `range_min`/`range_max` 决定了归一化映射的范围
- **当前状态：校准文件为 range 0-4095，等于无裁剪**

#### 第三层：相对位移限幅 (max_relative_target)

- 代码位置：`robots/utils.py` → `ensure_safe_goal_position()`
- 限制每一步指令的最大位移增量，防止突然大幅运动
- 在 `SO101FollowerConfig` 和 `XLerobotConfig` 中均有配置项：

```python
# config_so101_follower.py
max_relative_target: float | dict[str, float] | None = None
```

- **当前状态：默认值为 `None`，即未启用**

#### 不存在的机制

| 缺失的安全机制 | 说明 |
|:---:|:---:|
| 逆运动学自碰撞检测 | 原仓库无运动学模型级别的碰撞检查 |
| 末端执行器工作空间限制 | 无笛卡尔空间边界约束 |
| 关节角度绝对上下限 | 仅依赖硬件寄存器和校准文件 |
| 关节间耦合约束 | 无 "肘部+肩部组合角度" 之类的联合约束 |

### 7.2 辅助工具

原仓库提供了一个手动探测关节极限的脚本：

```bash
lerobot-find-joint-limits \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM2
```

该脚本通过遥操作让机械臂运动 30 秒，记录关节位置和末端位置的 min/max 值。但这只是一个数据收集工具，不会自动写入限位。

---

## 8. 自碰撞风险评估

### 当前风险：高

| 风险项 | 严重程度 | 原因 |
|:---:|:---:|:---:|
| 关节全量程旋转 | **高** | 硬件限位 0-4095，关节可转满 360° |
| 臂自碰撞 | **高** | 无任何软件/硬件限制，肘部可能撞到肩部或底座 |
| 夹爪过扭矩 | **中** | 夹爪 Torque_Limit=500 (其他=1000)，有一定保护 |
| 底盘失控 | **低** | 底盘使用速度模式，有软件限速 |

### 高风险关节组合

- **shoulder_lift (ID2) + elbow_flex (ID3)**：肘部和肩部如果同向弯曲，前臂会撞到底座或自身
- **wrist_flex (ID4)**：腕部过度弯曲可能导致线缆拉扯或撞到前臂
- **shoulder_pan (ID1)**：底座旋转无限位，可能缠绕线缆

---

## 9. 建议操作

### 第一步：运行校准 (写入硬件限位)

```bash
conda activate new-lerobot
cd ~/lerobot-MakerMods

# 校准左臂 — 脚本会自动探测每个关节的机械极限并写入 EEPROM
python -m lerobot.scripts.lerobot_measure_feetech_ranges \
    --port /dev/ttyACM0 --save --robot-id arm_left

# 校准右臂
python -m lerobot.scripts.lerobot_measure_feetech_ranges \
    --port /dev/ttyACM1 --save --robot-id arm_right
```

校准完成后, 脚本会：
- 探测每个关节的实际机械极限 (range_min / range_max)
- 计算并写入 Homing_Offset 到舵机 EEPROM
- 将 Min_Position_Limit / Max_Position_Limit 写入硬件
- 保存校准文件到 `~/.cache/huggingface/lerobot/calibration/robots/so_follower/`

### 第二步：启用 max_relative_target (限制速度突变)

在使用 SO101Follower 或 XLerobot 配置时设置 `max_relative_target`，例如：

```python
config = SO101FollowerConfig(
    port="/dev/ttyACM0",
    max_relative_target=5.0,  # 每步最大位移 5 个归一化单位
)
```

### 第三步 (可选)：在 robot_skills.py 中添加软件安全限位

在 `robot_skills.py` 或 `config.py` 中定义各关节的安全范围，作为额外保护层。

---

## 附录: 原始数据

详细寄存器值保存在: `dexproject/servo_limits_raw.json`
