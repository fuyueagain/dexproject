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

        subtasks = [SubTask(index=i, description=d) for i, d in enumerate(subtask_descs)]

        plan_text = "## 任务规划\n" + "\n".join(
            f"{i+1}. {st.description}" for i, st in enumerate(subtasks)
        )
        await self._emit({"type": "chat", "role": "assistant", "content": plan_text})

        return subtasks

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
                result = await self._execute_tool(fc)
                result_str = json.dumps(result, ensure_ascii=False)

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
                    subtask.completed = True
                    break
                if fc.name == "finish_task":
                    subtask.completed = True
                    self._running = False
                    break

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
