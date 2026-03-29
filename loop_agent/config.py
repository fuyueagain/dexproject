"""Loop Agent 全局配置"""

# ── LLM API ──
LLM_API_URL = "https://www.right.codes/codex/v1/responses"
LLM_API_KEY = "sk-39243fd6337142e09fbe88423260e518"
LLM_MODEL = "gpt-5.4"
LLM_FALLBACK_MODEL = "gpt-5.2"
LLM_TIMEOUT = 120  # 秒
LLM_MAX_TOKENS = 4096

# ── 服务器 ──
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8080

# ── Agent 参数 ──
MAX_STEPS_PER_SUBTASK = 50
MAX_DELTA_PER_STEP = 5.0  # 归一化单位, 防止单步过大
MAX_CARTESIAN_DELTA = 0.05  # 笛卡尔空间单步最大增量 (米), 5cm
OBSERVATION_RESIZE = (320, 240)  # 发送给 LLM 的图片尺寸 (节省 token)

# ── 关节描述 (嵌入 system prompt 供 LLM 理解) ──
JOINT_DESCRIPTION = """
### 臂关节 (左臂和右臂结构相同, 各6个关节)
归一化范围: [-100, 100] (gripper 为 [0, 100])

| 关节名 | 功能 | 正值方向 | 负值方向 |
|--------|------|----------|----------|
| shoulder_pan | 肩部旋转 | 向一侧旋转 | 向另一侧旋转 |
| shoulder_lift | 肩部俯仰 | 抬臂 | 放臂 |
| elbow_flex | 肘部弯曲 | 弯曲 | 伸展 |
| wrist_flex | 腕部俯仰 | 上翘 | 下压 |
| wrist_roll | 腕部旋转 | 顺时针 | 逆时针 |
| gripper | 夹爪 | 0=闭合, 100=张开 | — |

> 注意: 正负方向的具体含义需要通过实际观察确认。每次调整后请仔细观察相机画面,
> 确认实际运动方向，如发现与预期相反，立即反转并记住。

### 头部云台
| 关节名 | 功能 | 正值方向 |
|--------|------|----------|
| head_pan | 水平旋转 | 正=左转 |
| head_tilt | 垂直俯仰 | 正=上抬 |

### 底盘
| 参数 | 功能 |
|------|------|
| x | 前进(正)/后退(负) m/s |
| y | 左移(正)/右移(负) m/s |
| theta | 逆时针旋转 deg/s |

### 笛卡尔空间位置控制 (IK)
除了关节空间增量控制外，还支持笛卡尔空间(xyz)位置增量控制。
工作坐标系定义 (相对于臂基座):
| 轴 | 方向 | 正值含义 |
|----|------|----------|
| x | 前后 | 正=前伸 |
| y | 左右 | 正=左移 |
| z | 上下 | 正=上抬 |

- 单步增量限制: ±5cm (0.05m)
- 适合精细位置调整，如对准目标、微调抓取位置
- 使用 move_arm_cartesian_delta 进行增量移动
- 使用 get_cartesian_pose 查看末端当前位置
- 使用 get_feasible_range 查询当前位姿下的可达范围
"""

SYSTEM_PROMPT = f"""你是 YuanClaw Loop Agent，一个控制 XLeRobot 双臂机器人的视觉运动闭环智能体。

## 硬件配置
- 双臂: 左臂 + 右臂, 各6个关节(shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll) + 夹爪(gripper)
- 头部云台: pan(水平) + tilt(垂直)
- 底盘: 三轮全向移动
- 相机: cam_a(头部相机, 云台控制), cam_b(右腕相机, 近景视角)

## 关节控制说明
{JOINT_DESCRIPTION}

## 视觉伺服原则
1. **小步增量**: 每次关节调整量应很小 (1~5个归一化单位)，避免大幅度运动。
2. **观察-调整循环**: 每执行一步后观察相机画面，确认运动效果。
3. **方向确认**: 如果某个关节的运动方向与预期相反，立即反转方向并记住正确映射。
4. **目标追踪**: 持续关注目标物体在画面中的位置变化:
   - 是否变近/变远
   - 是否对准夹爪
   - 夹爪是否成功抓住物体
5. **安全优先**: 如果检测到异常（碰撞、物体掉落等），立即停止并汇报。

## 两种控制模式选择
- **关节空间控制 (move_arm)**: 直接控制各关节角度增量，适合大范围姿态调整。
- **笛卡尔空间控制 (move_arm_cartesian_delta)**: 控制末端在 xyz 方向的位移(米)，适合精细定位和对准。
  - 建议每次增量 0.005~0.03m (5mm~3cm)
  - 先用 get_cartesian_pose 了解当前位置，再决定增量方向和大小
  - 如果位置不可达，IK 会返回最接近的结果
- **直接方向口令优先映射**:
  - “抬起右臂” → 优先调用 `move_arm_cartesian_delta(arm="right", dz>0)`
  - “放下右臂” → 优先调用 `move_arm_cartesian_delta(arm="right", dz<0)`
  - “抬起左臂” → 优先调用 `move_arm_cartesian_delta(arm="left", dz>0)`
  - “放下左臂” → 优先调用 `move_arm_cartesian_delta(arm="left", dz<0)`
  - “前伸/后缩/左移/右移” 这类末端方向明确的口令，也优先使用 `move_arm_cartesian_delta`
  - 只有在任务明确要求姿态调整，或笛卡尔移动不足以完成目标时，才优先考虑 `move_arm`

## 执行流程
当收到任务指令时:
1. 分析任务，拆解为有序的子任务列表
2. 对每个子任务，执行"感知→决策→行动"循环:
   a. 观察当前相机画面和关节状态
   b. 分析当前子任务进度
   c. 决定下一步增量动作 (调用工具)
   d. 如果子任务完成，调用 finish_subtask 切换到下一个
3. 所有子任务完成后，调用 finish_task

## 重要提醒
- 你每次可以看到两个相机的实时画面和所有关节的当前位置
- 请主动描述你在画面中看到的内容，以及你的推理过程
- 每次只执行一个动作，然后等待下一次观察
- 如果不确定方向，先做一个很小的试探动作（delta=1），观察效果后再决定
- 对于涉及“左臂/右臂/夹爪/抓取/放置/搬运”的子任务，不能只通过 move_head 完成
- 只有在对应机械臂或夹爪已经实际执行成功后，才能调用 finish_subtask
"""

TOOL_DEFINITIONS = [
    {
        "type": "function",
        "name": "move_arm",
        "description": "对指定手臂的关节进行增量控制。适合姿态调整；像“抬起右臂/前伸”这种末端方向明确的命令，优先改用 move_arm_cartesian_delta。delta 值范围建议 [-5, 5]。",
        "parameters": {
            "type": "object",
            "properties": {
                "arm": {
                    "type": "string",
                    "enum": ["left", "right"],
                    "description": "要控制的手臂"
                },
                "deltas": {
                    "type": "object",
                    "description": "关节增量, key 为关节名, value 为增量值",
                    "properties": {
                        "shoulder_pan": {"type": "number"},
                        "shoulder_lift": {"type": "number"},
                        "elbow_flex": {"type": "number"},
                        "wrist_flex": {"type": "number"},
                        "wrist_roll": {"type": "number"},
                    }
                }
            },
            "required": ["arm", "deltas"]
        }
    },
    {
        "type": "function",
        "name": "set_gripper",
        "description": "设置指定手臂的夹爪开合度。0=完全闭合, 100=完全张开。",
        "parameters": {
            "type": "object",
            "properties": {
                "arm": {
                    "type": "string",
                    "enum": ["left", "right"],
                    "description": "要控制的手臂"
                },
                "openness": {
                    "type": "number",
                    "minimum": 0,
                    "maximum": 100,
                    "description": "夹爪开合度 (0=闭合, 100=张开)"
                }
            },
            "required": ["arm", "openness"]
        }
    },
    {
        "type": "function",
        "name": "move_head",
        "description": "移动头部云台 (增量控制, 单位: 度)。",
        "parameters": {
            "type": "object",
            "properties": {
                "pan_delta": {"type": "number", "description": "水平增量 (正=左转)"},
                "tilt_delta": {"type": "number", "description": "垂直增量 (正=上抬)"}
            },
            "required": []
        }
    },
    {
        "type": "function",
        "name": "move_base",
        "description": "控制底盘移动一段时间。",
        "parameters": {
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "前进速度 m/s (正=前进)"},
                "y": {"type": "number", "description": "横移速度 m/s (正=左移)"},
                "theta": {"type": "number", "description": "旋转速度 deg/s (正=逆时针)"},
                "duration": {"type": "number", "description": "持续时间 (秒)", "default": 0.5}
            },
            "required": []
        }
    },
    {
        "type": "function",
        "name": "move_arm_cartesian_delta",
        "description": "笛卡尔空间增量移动手臂末端。单位: 米。工作坐标系: x=前后, y=左右, z=上下。像“抬起右臂/放下右臂/前伸/后缩/左移/右移”这类直接动作命令应优先使用它。每轴单步限制 ±0.05m。",
        "parameters": {
            "type": "object",
            "properties": {
                "arm": {
                    "type": "string",
                    "enum": ["left", "right"],
                    "description": "要控制的手臂"
                },
                "dx": {
                    "type": "number",
                    "description": "前后增量(米), 正=前伸, 负=后缩"
                },
                "dy": {
                    "type": "number",
                    "description": "左右增量(米), 正=左移, 负=右移"
                },
                "dz": {
                    "type": "number",
                    "description": "上下增量(米), 正=上抬, 负=下压"
                },
                "duration": {
                    "type": "number",
                    "description": "移动时长(秒), 默认 2.0",
                    "default": 2.0
                }
            },
            "required": ["arm"]
        }
    },
    {
        "type": "function",
        "name": "move_arm_cartesian",
        "description": "笛卡尔空间绝对位置移动手臂末端。单位: 米。工作坐标系: x=前后, y=左右, z=上下。",
        "parameters": {
            "type": "object",
            "properties": {
                "arm": {
                    "type": "string",
                    "enum": ["left", "right"],
                    "description": "要控制的手臂"
                },
                "x": {"type": "number", "description": "目标 x (米, 前方)"},
                "y": {"type": "number", "description": "目标 y (米, 左方)"},
                "z": {"type": "number", "description": "目标 z (米, 上方)"},
                "gripper": {
                    "type": "number", "minimum": 0, "maximum": 100,
                    "description": "夹爪开合度, 不传则保持当前"
                },
                "duration": {
                    "type": "number",
                    "description": "移动时长(秒), 默认 2.0",
                    "default": 2.0
                }
            },
            "required": ["arm", "x", "y", "z"]
        }
    },
    {
        "type": "function",
        "name": "get_cartesian_pose",
        "description": "获取指定手臂末端的笛卡尔空间位姿 (位置 + 姿态)。",
        "parameters": {
            "type": "object",
            "properties": {
                "arm": {
                    "type": "string",
                    "enum": ["left", "right"],
                    "description": "要查询的手臂"
                }
            },
            "required": ["arm"]
        }
    },
    {
        "type": "function",
        "name": "get_feasible_range",
        "description": "获取指定手臂当前位姿下各轴 (dx/dy/dz) 的可行增量范围。",
        "parameters": {
            "type": "object",
            "properties": {
                "arm": {
                    "type": "string",
                    "enum": ["left", "right"],
                    "description": "要查询的手臂"
                }
            },
            "required": ["arm"]
        }
    },
    {
        "type": "function",
        "name": "finish_subtask",
        "description": "标记当前子任务为已完成，切换到下一个子任务。只有当当前子任务的目标已经真正达成时才能调用；如果子任务涉及机械臂/夹爪操作，不能仅靠头部扫描后调用。",
        "parameters": {
            "type": "object",
            "properties": {
                "reason": {"type": "string", "description": "完成原因的简要说明"}
            },
            "required": ["reason"]
        }
    },
    {
        "type": "function",
        "name": "finish_task",
        "description": "标记整个任务为已完成。",
        "parameters": {
            "type": "object",
            "properties": {
                "summary": {"type": "string", "description": "任务完成总结"}
            },
            "required": ["summary"]
        }
    },
]
