#!/usr/bin/env python3
"""
双臂视觉伺服任务执行脚本
- 左臂抓取白色盒子 → 移到右侧 → 松开
- 右臂抓取 → 移到更右侧
"""

import asyncio
import json
import logging
import sys
import time
from pathlib import Path

# Setup paths
sys.path.insert(0, str(Path.home() / "dexproject"))
sys.path.insert(0, str(Path(__file__).parent))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("run_task")

TASK_DESCRIPTION = """双臂接力抓取任务：
1. 左臂夹爪抓住白色盒子
2. 左臂将盒子移到右侧区域
3. 左臂松开，放下盒子
4. 右臂夹爪抓住该盒子
5. 右臂将盒子移到更右侧区域"""


async def main():
    from loop_agent.agent import LoopAgent
    from loop_agent.llm_client import LLMClient
    from loop_agent.skills import RobotSkillsWrapper

    # 事件追踪
    class ProgressTracker:
        def __init__(self):
            self.step_count = 0
            self.last_subtask = -1
            self.events = []
            self.task_completed = False

        async def callback(self, event: dict):
            etype = event.get("type", "")
            self.events.append(event)

            if etype == "step_start":
                self.step_count += 1
                step = event.get("step", "?")
                print(f"\n📍 步骤 {step}")

            elif etype == "agent_thought":
                content = event.get("content", "")
                if content:
                    print(f"💭 思考: {content[:200]}...")

            elif etype == "tool_result":
                name = event.get("name", "")
                result = event.get("result", {})
                args = event.get("arguments", {})
                print(f"🔧 工具: {name}({json.dumps(args, ensure_ascii=False)[:80]}) → {json.dumps(result, ensure_ascii=False)[:150]}")

            elif etype == "subtask_start":
                idx = event.get("index", "?")
                desc = event.get("description", "")
                self.last_subtask = idx
                print(f"\n{'='*60}")
                print(f"🎯 子任务 {idx + 1}: {desc}")
                print(f"{'='*60}")
                await self._report_progress(f"🎯 开始子任务 {idx + 1}: {desc}")

            elif etype == "agent_status":
                state = event.get("state", "")
                total_steps = event.get("total_steps", 0)
                current_idx = event.get("current_subtask_idx", -1)
                subtasks = event.get("subtasks", [])

                completed = sum(1 for s in subtasks if s.get("completed", False))
                total = len(subtasks)
                print(f"📊 状态: {state} | 已完成子任务: {completed}/{total} | 总步数: {total_steps}")

            elif etype == "chat":
                role = event.get("role", "")
                content = event.get("content", "")
                if content:
                    print(f"💬 {role}: {content[:200]}")

            elif etype == "error":
                msg = event.get("message", "")
                print(f"❌ 错误: {msg}")
                await self._report_progress(f"❌ 错误: {msg}")

            elif etype == "observation":
                step = event.get("step", "?")
                images_count = len(event.get("images", []))
                comparisons_count = len(event.get("comparisons", []))
                print(f"📷 观测: step={step}, 图片={images_count}, 对比图={comparisons_count}")

            # 每50步报告一次
            if self.step_count > 0 and self.step_count % 50 == 0:
                await self._report_progress(f"📍 进度报告: 第{self.step_count}步, 当前子任务={self.last_subtask + 1}")

        async def _report_progress(self, msg: str):
            """向主会话报告进度"""
            try:
                from openclaw_core import sessions
                await sessions.send(
                    sessionKey="main",
                    message=f"[双臂任务进度] {msg}"
                )
            except Exception as e:
                print(f"[报告失败] {e}")

        async def final_report(self):
            """最终报告"""
            completed_tasks = [s for s in self.events if s.get("type") == "subtask_start"]
            final_events = [e for e in self.events if e.get("type") in ("chat", "tool_result", "error")]

            summary = f"""
=== 双臂任务执行完成 ===
总步骤数: {self.step_count}
子任务数: {len(completed_tasks)}
最后状态: {self.events[-1].get('state', 'unknown') if self.events else 'unknown'}
"""
            print(summary)
            try:
                from openclaw_core import sessions
                await sessions.send(
                    sessionKey="main",
                    message=f"[双臂任务完成] {summary}"
                )
            except Exception as e:
                print(f"[最终报告失败] {e}")

    tracker = ProgressTracker()

    # 初始化组件
    print("🔧 初始化 LoopAgent...")
    skills = RobotSkillsWrapper()
    llm = LLMClient()
    agent = LoopAgent(skills, llm)
    agent.set_callback(tracker.callback)

    # 连接机器人
    print("🔌 连接机器人...")
    try:
        connect_result = skills.connect()
        print(f"连接结果: {connect_result}")
    except Exception as e:
        print(f"连接失败: {e}")
        print("⚠️ 机器人未连接，将模拟执行流程（不发送实际控制指令）")

    # 检查相机状态
    cameras = skills.camera_streamer.available_cameras
    print(f"📷 可用相机: {cameras}")

    if not cameras:
        print("⚠️ 没有可用相机，任务可能无法正常执行")

    # 执行任务
    print(f"\n🚀 开始执行任务:\n{TASK_DESCRIPTION}\n")
    await tracker._report_progress(f"开始执行任务，共5个子任务")

    try:
        await agent.run_task(TASK_DESCRIPTION)
    except Exception as e:
        print(f"❌ 任务执行异常: {e}")
        import traceback
        traceback.print_exc()

    await tracker.final_report()

    # 断开连接
    print("🔌 断开机器人连接...")
    skills.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
