"""Loop Agent 主控 — 感知→决策→行动 循环"""

import asyncio
import json
import logging
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Coroutine, Optional

from .config import MAX_STEPS_PER_SUBTASK, SYSTEM_PROMPT, TOOL_DEFINITIONS
from .llm_client import FunctionCall, LLMClient, StreamEvent
from .skills import RobotSkillsWrapper

logger = logging.getLogger("loop_agent.agent")


class AgentState(str, Enum):
    IDLE = "idle"
    PLANNING = "planning"
    EXECUTING = "executing"
    WAITING = "waiting"
    COMPLETED = "completed"
    ERROR = "error"
    CHATTING = "chatting"


@dataclass
class SubTask:
    index: int
    description: str
    completed: bool = False
    steps_taken: int = 0
    requires_arm_action: bool = False
    required_arm: Optional[str] = None
    arm_action_completed: bool = False


@dataclass
class AgentStatus:
    state: AgentState = AgentState.IDLE
    current_task: str = ""
    subtasks: list[SubTask] = field(default_factory=list)
    current_subtask_idx: int = 0
    total_steps: int = 0
    last_thought: str = ""
    last_action: str = ""

    def to_dict(self) -> dict:
        return {
            "state": self.state.value,
            "current_task": self.current_task,
            "subtasks": [
                {"index": st.index, "description": st.description,
                 "completed": st.completed, "steps": st.steps_taken}
                for st in self.subtasks
            ],
            "current_subtask_idx": self.current_subtask_idx,
            "total_steps": self.total_steps,
            "last_thought": self.last_thought,
            "last_action": self.last_action,
        }


EventCallback = Callable[[dict], Coroutine[Any, Any, None]]


class LoopAgent:
    def __init__(self, skills: RobotSkillsWrapper, llm: LLMClient):
        self.skills = skills
        self.llm = llm
        self.status = AgentStatus()
        self._callback: Optional[EventCallback] = None
        self._running = False
        self._conversation: list[dict] = []

    def set_callback(self, callback: EventCallback):
        self._callback = callback

    async def _emit(self, event: dict):
        if self._callback:
            try:
                await self._callback(event)
            except Exception as e:
                logger.warning(f"事件回调失败: {e}")

    def stop(self):
        self._running = False

    async def chat(self, user_message: str):
        """处理普通聊天（非任务模式）"""
        self.status.state = AgentState.CHATTING
        await self._emit({"type": "agent_status", **self.status.to_dict()})

        self._conversation.append(
            self.llm.build_user_message(user_message)
        )

        full_text = ""
        async for event in self.llm.create_response_stream(
            input_messages=self._conversation,
            instructions=SYSTEM_PROMPT,
        ):
            if event.type == "text_delta":
                full_text += event.content
                await self._emit({"type": "stream_delta", "content": event.content})
            elif event.type == "done":
                break
            elif event.type == "error":
                await self._emit({"type": "error", "message": event.content})
                break

        if full_text:
            self._conversation.append({
                "type": "message",
                "role": "assistant",
                "content": [{"type": "output_text", "text": full_text}],
            })
            await self._emit({"type": "chat", "role": "assistant", "content": full_text})

        self.status.state = AgentState.IDLE
        await self._emit({"type": "agent_status", **self.status.to_dict()})

    async def run_task(self, task_description: str):
        """执行机器人任务（完整的 loop agent 流程）"""
        self._running = True
        self.status = AgentStatus(
            state=AgentState.PLANNING,
            current_task=task_description,
        )
        self._conversation = []
        await self._emit({"type": "agent_status", **self.status.to_dict()})

        try:
            direct_action = self._parse_direct_action(task_description)
            if direct_action:
                await self._run_direct_action(task_description, direct_action)
                return

            # 阶段 1: 任务规划
            subtasks = await self._plan_subtasks(task_description)
            if not subtasks:
                await self._emit({"type": "error", "message": "任务规划失败"})
                return

            self.status.subtasks = subtasks
            self.status.state = AgentState.EXECUTING
            await self._emit({"type": "agent_status", **self.status.to_dict()})

            # 阶段 2: 逐个执行子任务
            for i, subtask in enumerate(subtasks):
                if not self._running:
                    await self._emit({"type": "chat", "role": "assistant",
                                      "content": "任务已被用户中止。"})
                    break

                self.status.current_subtask_idx = i
                await self._emit({
                    "type": "subtask_start",
                    "index": i,
                    "description": subtask.description,
                })

                await self._execute_subtask(subtask)

                if not subtask.completed and self._running:
                    await self._emit({
                        "type": "chat", "role": "assistant",
                        "content": f"子任务 {i+1} 达到最大步数限制，自动切换到下一个。",
                    })

            self.status.state = AgentState.COMPLETED
            await self._emit({"type": "agent_status", **self.status.to_dict()})
            await self._emit({
                "type": "chat", "role": "assistant",
                "content": "所有子任务执行完毕。",
            })

        except Exception as e:
            self.status.state = AgentState.ERROR
            logger.error(f"任务执行异常: {e}", exc_info=True)
            await self._emit({"type": "error", "message": str(e)})
        finally:
            self._running = False
            self.status.state = AgentState.IDLE
            await self._emit({"type": "agent_status", **self.status.to_dict()})

    async def _plan_subtasks(self, task: str) -> list[SubTask]:
        """调用 LLM 将任务拆解为子任务"""
        await self._emit({"type": "chat", "role": "assistant",
                          "content": "正在分析任务，拆解子任务..."})

        plan_prompt = (
            f"请将以下机器人任务拆解为有序的子任务列表。\n\n"
            f"任务: {task}\n\n"
            "如果任务涉及左臂/右臂/夹爪/抓取/放置/搬运，子任务中必须包含实际的机械臂或夹爪动作。\n"
            "不要把任务拆成只有头部扫描/观察的子任务序列；云台扫描只能作为辅助观察，不能替代机械臂操作。\n\n"
            f"请严格按以下 JSON 格式返回（只返回 JSON，不要其他内容）:\n"
            f'{{"subtasks": ["子任务1描述", "子任务2描述", ...]}}'
        )

        messages = [self.llm.build_user_message(plan_prompt)]
        response = await self.llm.create_response(
            input_messages=messages,
            instructions=SYSTEM_PROMPT,
        )

        if response.error:
            await self._emit({"type": "error", "message": f"规划失败: {response.error}"})
            return []

        try:
            text = response.text.strip()
            if "```" in text:
                text = text.split("```")[1]
                if text.startswith("json"):
                    text = text[4:]
                text = text.strip()
            data = json.loads(text)
            subtask_descs = data.get("subtasks", [])
        except (json.JSONDecodeError, KeyError):
            subtask_descs = [task]
            await self._emit({"type": "chat", "role": "assistant",
                              "content": "无法解析子任务列表，将任务作为单个子任务执行。"})

        subtasks = []
        for i, desc in enumerate(subtask_descs):
            requires_arm_action, required_arm = self._infer_subtask_arm_requirement(task, desc)
            subtasks.append(SubTask(
                index=i,
                description=desc,
                requires_arm_action=requires_arm_action,
                required_arm=required_arm,
            ))

        if self._task_requires_arm_manipulation(task) and not any(
            st.requires_arm_action for st in subtasks
        ):
            fallback_arm = self._infer_arm_from_text(task)
            subtasks = [SubTask(
                index=0,
                description=task,
                requires_arm_action=True,
                required_arm=fallback_arm,
            )]
            await self._emit({
                "type": "chat",
                "role": "assistant",
                "content": "规划结果没有包含实际机械臂动作，已回退为直接按原任务执行，避免只做云台扫描。",
            })

        plan_text = "## 任务规划\n" + "\n".join(
            f"{i+1}. {st.description}" for i, st in enumerate(subtasks)
        )
        await self._emit({"type": "chat", "role": "assistant", "content": plan_text})

        return subtasks

    @staticmethod
    def _parse_direct_action(task: str) -> Optional[dict]:
        """为高频简单中文动作提供确定性执行，绕过 LLM 不稳定的工具选择。"""
        text = task.strip()
        arm = None
        if "右臂" in text:
            arm = "right"
        elif "左臂" in text:
            arm = "left"

        if arm is None:
            return None

        duration = 1.5
        if any(k in text for k in ("抬起", "抬高", "上抬", "举起")):
            return {
                "summary": f"直接执行{('右臂' if arm == 'right' else '左臂')}上抬",
                "primary_tool": {
                    "name": "move_arm_cartesian_delta",
                    "arguments": {"arm": arm, "dz": 0.02, "duration": duration},
                },
                "fallback_tool": {
                    "name": "move_arm",
                    "arguments": {"arm": arm, "deltas": {"shoulder_lift": 4.0}},
                },
            }

        if any(k in text for k in ("放下", "降低", "下压", "下移")):
            return {
                "summary": f"直接执行{('右臂' if arm == 'right' else '左臂')}下压",
                "primary_tool": {
                    "name": "move_arm_cartesian_delta",
                    "arguments": {"arm": arm, "dz": -0.02, "duration": duration},
                },
                "fallback_tool": {
                    "name": "move_arm",
                    "arguments": {"arm": arm, "deltas": {"shoulder_lift": -4.0}},
                },
            }

        return None

    async def _run_direct_action(self, task_description: str, action: dict) -> None:
        """执行直接动作命令，并在必要时回退到更底层的关节控制。"""
        subtask = SubTask(index=0, description=action["summary"])
        self.status.subtasks = [subtask]
        self.status.state = AgentState.EXECUTING
        self.status.current_subtask_idx = 0
        await self._emit({"type": "agent_status", **self.status.to_dict()})
        await self._emit({
            "type": "subtask_start",
            "index": 0,
            "description": subtask.description,
        })
        await self._emit({
            "type": "chat",
            "role": "assistant",
            "content": f"检测到简单直接动作指令，跳过规划，直接执行：{task_description}",
        })

        for tool_spec in (action["primary_tool"], action.get("fallback_tool")):
            if not tool_spec:
                continue

            call_id = str(uuid.uuid4())
            fc = FunctionCall(
                call_id=call_id,
                name=tool_spec["name"],
                arguments=json.dumps(tool_spec["arguments"], ensure_ascii=False),
            )

            await self._emit({
                "type": "tool_start",
                "name": fc.name,
                "call_id": call_id,
            })
            await self._emit({
                "type": "tool_ready",
                "name": fc.name,
                "call_id": call_id,
                "arguments": fc.arguments,
            })

            result = await self._execute_tool(fc)
            self.status.total_steps += 1
            self.status.last_action = f"{fc.name}({fc.arguments})"
            await self._emit({
                "type": "tool_result",
                "name": fc.name,
                "call_id": call_id,
                "arguments": fc.parsed_arguments(),
                "result": result,
            })

            if result.get("success"):
                subtask.completed = True
                self.status.state = AgentState.COMPLETED
                await self._emit({"type": "agent_status", **self.status.to_dict()})
                await self._emit({
                    "type": "chat",
                    "role": "assistant",
                    "content": f"已执行完成：{action['summary']}",
                })
                return

            logger.warning(
                "直接动作执行失败，准备尝试回退工具: %s -> %s",
                fc.name, result,
            )

        self.status.state = AgentState.ERROR
        await self._emit({"type": "agent_status", **self.status.to_dict()})
        await self._emit({
            "type": "error",
            "message": f"直接动作执行失败: {task_description}",
        })

    @staticmethod
    def _build_direct_motion_hint(task: str, subtask: str) -> str:
        """为简单自然语言动作生成稳定的工具调用提示。"""
        text = f"{task} {subtask}"
        arm = None
        if "右臂" in text:
            arm = "right"
        elif "左臂" in text:
            arm = "left"

        if arm is None:
            return ""

        directions = []
        if any(k in text for k in ("抬起", "抬高", "上抬", "举起")):
            directions.append(f"- 这是明确的抬臂命令，优先调用 `move_arm_cartesian_delta(arm=\"{arm}\", dz=0.01~0.03)`。")
        if any(k in text for k in ("放下", "降低", "下压", "下移")):
            directions.append(f"- 这是明确的下压命令，优先调用 `move_arm_cartesian_delta(arm=\"{arm}\", dz=-0.01~-0.03)`。")
        if any(k in text for k in ("前伸", "伸出", "向前")):
            directions.append(f"- 这是明确的前伸命令，优先调用 `move_arm_cartesian_delta(arm=\"{arm}\", dx=0.01~0.03)`。")
        if any(k in text for k in ("后缩", "缩回", "向后")):
            directions.append(f"- 这是明确的后缩命令，优先调用 `move_arm_cartesian_delta(arm=\"{arm}\", dx=-0.01~-0.03)`。")
        if any(k in text for k in ("左移", "往左", "向左")):
            directions.append(f"- 这是明确的左移命令，优先调用 `move_arm_cartesian_delta(arm=\"{arm}\", dy=0.01~0.03)`。")
        if any(k in text for k in ("右移", "往右", "向右")):
            directions.append(f"- 这是明确的右移命令，优先调用 `move_arm_cartesian_delta(arm=\"{arm}\", dy=-0.01~-0.03)`。")

        if not directions:
            return ""

        return "## 直接动作映射提示\n" + "\n".join(directions)

    @staticmethod
    def _infer_arm_from_text(text: str) -> Optional[str]:
        has_left = "左臂" in text
        has_right = "右臂" in text
        if has_left and not has_right:
            return "left"
        if has_right and not has_left:
            return "right"
        return None

    @classmethod
    def _task_requires_arm_manipulation(cls, task: str) -> bool:
        arm_terms = ("左臂", "右臂", "夹爪", "机械臂", "gripper", "arm")
        manipulation_terms = (
            "抓", "取", "拿", "放", "搬", "递", "抬", "举", "伸", "缩",
            "夹", "松开", "闭合", "张开", "对准", "靠近", "移动",
        )
        return (
            any(term in task for term in arm_terms)
            and any(term in task for term in manipulation_terms)
        )

    @classmethod
    def _infer_subtask_arm_requirement(cls, task: str, subtask: str) -> tuple[bool, Optional[str]]:
        subtext = subtask.strip()
        tasktext = task.strip()
        required_arm = cls._infer_arm_from_text(subtext) or cls._infer_arm_from_text(tasktext)

        arm_terms = ("左臂", "右臂", "夹爪", "机械臂", "gripper", "arm")
        manipulation_terms = (
            "抓", "取", "拿", "放", "搬", "递", "抬", "举", "伸", "缩",
            "夹", "松开", "闭合", "张开", "对准", "靠近", "移动",
        )
        head_scan_terms = (
            "云台", "头部", "相机", "cam_a", "cam_b", "扫描",
            "观察", "查看", "寻找", "定位", "搜索",
        )

        sub_mentions_arm = any(term in subtext for term in arm_terms)
        sub_mentions_manipulation = any(term in subtext for term in manipulation_terms)
        sub_is_head_only = (
            any(term in subtext for term in head_scan_terms)
            and not sub_mentions_arm
        )

        if sub_is_head_only:
            return False, required_arm

        if sub_mentions_arm and sub_mentions_manipulation:
            return True, required_arm

        if sub_mentions_manipulation and cls._task_requires_arm_manipulation(tasktext):
            return True, required_arm

        return False, required_arm

    @staticmethod
    def _tool_counts_as_arm_action(subtask: SubTask, fc: FunctionCall, result: dict) -> bool:
        if not result.get("success"):
            return False
        if fc.name not in (
            "move_arm",
            "move_arm_cartesian_delta",
            "move_arm_cartesian",
            "set_gripper",
        ):
            return False

        args = fc.parsed_arguments()
        arm = args.get("arm")
        if subtask.required_arm and arm and arm != subtask.required_arm:
            return False

        if fc.name == "move_arm":
            moved = result.get("moved", {})
            return any(abs(v) >= 0.5 for v in moved.values())

        if fc.name == "move_arm_cartesian_delta":
            actual = result.get("actual_delta", {})
            return any(abs(v) >= 0.003 for v in actual.values())

        if fc.name == "move_arm_cartesian":
            before = result.get("before", {})
            after = result.get("after", {})
            if not before or not after:
                return False
            deltas = [abs(after[k] - before[k]) for k in ("x", "y", "z")]
            return any(delta >= 0.003 for delta in deltas)

        return True

    async def _validate_finish_request(self, subtask: SubTask, fc: FunctionCall) -> Optional[dict]:
        if fc.name not in ("finish_subtask", "finish_task"):
            return None

        if not subtask.requires_arm_action or subtask.arm_action_completed:
            return None

        arm_label = {
            "left": "左臂",
            "right": "右臂",
            None: "机械臂/夹爪",
        }[subtask.required_arm]
        message = (
            f"当前子任务要求实际执行{arm_label}动作。"
            "仅移动头部云台或观察画面不能判定完成，请先成功调用对应手臂或夹爪工具。"
        )
        await self._emit({
            "type": "chat",
            "role": "assistant",
            "content": message,
        })
        return {"success": False, "error": message}

    async def _execute_subtask(self, subtask: SubTask):
        """执行单个子任务的感知-决策-行动循环"""
        while (not subtask.completed
               and subtask.steps_taken < MAX_STEPS_PER_SUBTASK
               and self._running):

            subtask.steps_taken += 1
            self.status.total_steps += 1
            step_n = subtask.steps_taken

            await self._emit({
                "type": "step_start",
                "subtask_idx": subtask.index,
                "step": step_n,
            })

            # 感知: 获取关节位置 + 相机画面 + 前后对比
            state_text, obs_images, comparisons = self.skills.get_observation_for_llm()
            images_b64 = [img["base64"] for img in obs_images]

            # 将观测图和对比图推送到前端
            await self._emit({
                "type": "observation",
                "step": step_n,
                "subtask_idx": subtask.index,
                "images": [
                    {"camera": img["camera"], "label": img["label"],
                     "base64": img["base64"]}
                    for img in obs_images
                ],
                "comparisons": comparisons,
            })

            # 已完成的子任务摘要
            done_summary = ""
            for st in self.status.subtasks[:subtask.index]:
                if st.completed:
                    done_summary += f"  ✓ {st.description}\n"

            # 构建完整上下文（每次都包含全部信息，不依赖对话历史）
            context_parts = [
                f"## 总任务目标\n{self.status.current_task}",
            ]
            if done_summary:
                context_parts.append(f"## 已完成的子任务\n{done_summary}")
            context_parts.append(
                f"## 当前子任务 ({subtask.index + 1}/{len(self.status.subtasks)})\n"
                f"{subtask.description}"
            )
            context_parts.append(f"## 当前步骤: {step_n}")
            context_parts.append(state_text)

            if images_b64:
                img_desc = ["图1: 头部相机 cam_a (云台上的全局视角相机)"]
                if len(images_b64) > 1:
                    img_desc.append("图2: 右腕相机 cam_b (右臂手腕上的近景相机)")
                context_parts.append("## 附带相机画面\n" + "\n".join(img_desc))
            else:
                context_parts.append("## 相机\n无可用画面")

            direct_hint = self._build_direct_motion_hint(
                self.status.current_task, subtask.description
            )
            if direct_hint:
                context_parts.append(direct_hint)

            if subtask.requires_arm_action:
                arm_label = {
                    "left": "左臂",
                    "right": "右臂",
                    None: "机械臂/夹爪",
                }[subtask.required_arm]
                context_parts.append(
                    "## 子任务完成约束\n"
                    f"当前子任务必须包含实际的{arm_label}动作。"
                    "仅移动头部云台、扫描画面或观察目标，不能判定此子任务完成。"
                    "在调用 finish_subtask 之前，必须至少成功执行一次对应手臂的 "
                    "move_arm / move_arm_cartesian_delta / move_arm_cartesian / set_gripper。"
                )

            context_parts.append(
                "## 指令\n"
                "请仔细观察相机画面和关节状态，描述你看到的内容，"
                "分析当前子任务进度，然后调用一个工具执行下一步动作。\n"
                "每次增量控制幅度建议 1~5。如果子任务已完成，调用 finish_subtask。"
            )
            context = "\n\n".join(context_parts)

            logger.info(
                f"[step {step_n}] 上下文: {len(context)}字, "
                f"图片: {len(images_b64)}张, "
                f"对比图: {len(comparisons)}张, "
                f"关节数据: {bool(state_text.strip())}"
            )

            # 构建发给 LLM 的消息列表:
            # - 历史消息只保留文本（去掉旧图片，节省 token）
            # - 只有最近 1 轮历史保留图片，当前步必带图片
            recent_history = self._conversation[-6:]
            messages = []
            for i, msg in enumerate(recent_history):
                is_recent = (i >= len(recent_history) - 2)
                if is_recent:
                    messages.append(msg)
                else:
                    messages.append(self._strip_images(msg))
            messages.append(self.llm.build_user_message(context, images_b64))

            # 决策（流式）
            thought_text = ""
            pending_calls_dict: dict[str, FunctionCall] = {}

            async for event in self.llm.create_response_stream(
                input_messages=messages,
                tools=TOOL_DEFINITIONS,
                instructions=SYSTEM_PROMPT,
            ):
                if event.type == "text_delta":
                    thought_text += event.content
                    await self._emit({"type": "stream_delta", "content": event.content})

                elif event.type == "tool_start":
                    fc = FunctionCall(call_id=event.call_id, name=event.tool_name)
                    pending_calls_dict[event.call_id] = fc
                    await self._emit({
                        "type": "tool_start",
                        "name": event.tool_name,
                        "call_id": event.call_id,
                    })

                elif event.type == "tool_delta":
                    if event.call_id in pending_calls_dict:
                        pending_calls_dict[event.call_id].arguments += event.content

                elif event.type == "tool_done":
                    if event.call_id in pending_calls_dict:
                        pending_calls_dict[event.call_id].arguments = event.content
                    await self._emit({
                        "type": "tool_ready",
                        "name": event.tool_name,
                        "call_id": event.call_id,
                        "arguments": event.content,
                    })

                elif event.type == "done":
                    break

                elif event.type == "error":
                    await self._emit({"type": "error", "message": event.content})
                    return

            if thought_text:
                self.status.last_thought = thought_text
                await self._emit({
                    "type": "agent_thought",
                    "step": step_n,
                    "content": thought_text,
                })

            # 行动：执行工具调用
            tool_summaries = []
            for call_id, fc in pending_calls_dict.items():
                finish_block = await self._validate_finish_request(subtask, fc)
                if finish_block is not None:
                    result = finish_block
                else:
                    result = await self._execute_tool(fc)
                result_str = json.dumps(result, ensure_ascii=False)

                if self._tool_counts_as_arm_action(subtask, fc, result):
                    subtask.arm_action_completed = True

                self.status.last_action = f"{fc.name}({fc.arguments}) → {result_str[:100]}"
                await self._emit({
                    "type": "tool_result",
                    "name": fc.name,
                    "call_id": call_id,
                    "arguments": fc.parsed_arguments(),
                    "result": result,
                })
                tool_summaries.append(
                    f"工具调用: {fc.name}({fc.arguments}) → {result_str[:200]}"
                )

                if fc.name == "finish_subtask":
                    if result.get("success"):
                        subtask.completed = True
                        break
                    continue
                if fc.name == "finish_task":
                    if result.get("success"):
                        subtask.completed = True
                        self._running = False
                        break
                    continue

            # 对话历史只存文本摘要(不存图片), 避免膨胀
            if tool_summaries or thought_text:
                summary = ""
                if thought_text:
                    summary += thought_text[:300] + "\n"
                summary += "\n".join(tool_summaries)
                self._conversation.append({
                    "type": "message",
                    "role": "assistant",
                    "content": [{"type": "output_text", "text": summary}],
                })

            # 如果没有工具调用，只有文本回复
            if not pending_calls_dict and thought_text:
                await self._emit({
                    "type": "chat", "role": "assistant",
                    "content": thought_text,
                })

            await self._emit({"type": "agent_status", **self.status.to_dict()})

            await asyncio.sleep(0.3)

            # 限制对话历史长度，防止 token 溢出
            if len(self._conversation) > 20:
                self._conversation = self._conversation[-10:]

    @staticmethod
    def _strip_images(msg: dict) -> dict:
        """从消息中去掉 base64 图片，只保留文本内容"""
        if msg.get("type") != "message":
            return msg
        content = msg.get("content", [])
        if not isinstance(content, list):
            return msg
        stripped = [
            item for item in content
            if item.get("type") not in ("input_image", "image_url")
        ]
        if len(stripped) == len(content):
            return msg
        return {**msg, "content": stripped}

    async def _execute_tool(self, fc: FunctionCall) -> dict:
        """执行单个工具调用"""
        args = fc.parsed_arguments()
        name = fc.name
        logger.info(f"执行工具: {name}({json.dumps(args, ensure_ascii=False)})")

        try:
            if name == "move_arm":
                arm = args.get("arm", "left")
                deltas = args.get("deltas", {})
                return self.skills.move_arm_delta(arm, deltas)

            elif name == "set_gripper":
                arm = args.get("arm", "left")
                openness = args.get("openness", 50)
                return self.skills.set_gripper(arm, openness)

            elif name == "move_head":
                pan_d = args.get("pan_delta", 0)
                tilt_d = args.get("tilt_delta", 0)
                return self.skills.move_head_delta(pan_d, tilt_d)

            elif name == "move_base":
                return self.skills.move_base(
                    x=args.get("x", 0), y=args.get("y", 0),
                    theta=args.get("theta", 0), duration=args.get("duration", 0.5),
                )

            elif name == "move_arm_cartesian_delta":
                arm = args.get("arm", "left")
                return self.skills.move_arm_cartesian_delta(
                    arm=arm,
                    dx=args.get("dx", 0.0), dy=args.get("dy", 0.0),
                    dz=args.get("dz", 0.0),
                    duration=args.get("duration", 2.0),
                )

            elif name == "move_arm_cartesian":
                arm = args.get("arm", "left")
                return self.skills.move_arm_cartesian(
                    arm=arm,
                    x=args["x"], y=args["y"], z=args["z"],
                    gripper=args.get("gripper"),
                    duration=args.get("duration", 2.0),
                )

            elif name == "get_cartesian_pose":
                arm = args.get("arm", "left")
                return self.skills.get_cartesian_pose(arm)

            elif name == "get_feasible_range":
                arm = args.get("arm", "left")
                return self.skills.get_feasible_range(arm)

            elif name == "finish_subtask":
                reason = args.get("reason", "")
                await self._emit({
                    "type": "chat", "role": "assistant",
                    "content": f"子任务完成: {reason}",
                })
                return {"success": True, "reason": reason}

            elif name == "finish_task":
                summary = args.get("summary", "")
                await self._emit({
                    "type": "chat", "role": "assistant",
                    "content": f"任务完成: {summary}",
                })
                return {"success": True, "summary": summary}

            else:
                return {"success": False, "error": f"未知工具: {name}"}

        except Exception as e:
            logger.error(f"工具执行失败: {name} - {e}")
            return {"success": False, "error": str(e)}
