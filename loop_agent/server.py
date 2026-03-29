"""FastAPI 后端 — WebSocket 聊天 + MJPEG 视频流 + Agent 控制"""

import asyncio
import json
import logging
import os
import time
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .agent import AgentState, LoopAgent
from .config import SERVER_HOST, SERVER_PORT
from .llm_client import LLMClient
from .skills import RobotSkillsWrapper

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("loop_agent.server")

STATIC_DIR = Path(__file__).parent / "static"

app = FastAPI(title="YuanClaw Loop Agent")

skills = RobotSkillsWrapper()
llm = LLMClient()
agent = LoopAgent(skills, llm)

connected_websockets: set[WebSocket] = set()


async def broadcast(event: dict):
    """广播事件到所有已连接的 WebSocket"""
    data = json.dumps(event, ensure_ascii=False)
    closed = set()
    for ws in connected_websockets:
        try:
            await ws.send_text(data)
        except Exception:
            closed.add(ws)
    connected_websockets.difference_update(closed)


agent.set_callback(broadcast)


# ── 静态文件 ──

@app.get("/")
async def index():
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/static/{file_path:path}")
async def static_file(file_path: str):
    fp = STATIC_DIR / file_path
    if fp.exists():
        return FileResponse(fp)
    return HTMLResponse("Not Found", status_code=404)


# ── MJPEG 视频流 ──

async def mjpeg_generator(camera: str):
    """MJPEG multipart 帧生成器"""
    while True:
        jpeg = skills.camera_streamer.get_frame_jpeg(camera, quality=70)
        if jpeg:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n"
                b"\r\n" + jpeg + b"\r\n"
            )
        else:
            placeholder = _make_placeholder(camera)
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(placeholder)).encode() + b"\r\n"
                b"\r\n" + placeholder + b"\r\n"
            )
        await asyncio.sleep(1 / 15)


def _make_placeholder(camera: str) -> bytes:
    """生成占位帧"""
    import cv2
    import numpy as np
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = (30, 30, 40)
    label = f"{camera} - No Signal"
    cv2.putText(img, label, (140, 250), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (100, 100, 120), 2)
    _, buf = cv2.imencode(".jpg", img)
    return buf.tobytes()


@app.get("/api/stream/{camera}")
async def video_stream(camera: str):
    return StreamingResponse(
        mjpeg_generator(camera),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


# ── 快照 API ──

@app.get("/api/snapshot/{camera}")
async def camera_snapshot(camera: str):
    jpeg = skills.camera_streamer.get_frame_jpeg(camera, quality=90)
    if jpeg:
        return StreamingResponse(iter([jpeg]), media_type="image/jpeg")
    return HTMLResponse("Camera not available", status_code=503)


# ── 机器人状态 API ──

@app.get("/api/robot/status")
async def robot_status():
    return {
        "connected": skills.connected,
        "positions": skills.get_all_positions(),
        "cameras": skills.camera_streamer.available_cameras,
        "agent_state": agent.status.state.value,
    }


# ── WebSocket ──

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    connected_websockets.add(ws)
    logger.info(f"WebSocket 已连接 (当前 {len(connected_websockets)} 个)")

    await ws.send_text(json.dumps({
        "type": "welcome",
        "robot_connected": skills.connected,
        "agent_status": agent.status.to_dict(),
    }, ensure_ascii=False))

    try:
        while True:
            raw = await ws.receive_text()
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                await ws.send_text(json.dumps({"type": "error", "message": "Invalid JSON"}))
                continue

            msg_type = msg.get("type", "")

            if msg_type == "chat":
                content = msg.get("content", "").strip()
                if not content:
                    continue

                await broadcast({"type": "chat", "role": "user", "content": content})

                if agent.status.state in (AgentState.IDLE, AgentState.COMPLETED, AgentState.ERROR):
                    is_task = _is_task_command(content)
                    if is_task:
                        if skills.connected:
                            asyncio.create_task(agent.run_task(content))
                        else:
                            await broadcast({
                                "type": "chat", "role": "assistant",
                                "content": "请先点击「连接机器人」再发送任务指令。",
                            })
                    else:
                        asyncio.create_task(agent.chat(content))
                else:
                    await broadcast({
                        "type": "chat", "role": "assistant",
                        "content": "Agent 正在执行中，请等待当前任务完成或点击停止。",
                    })

            elif msg_type == "connect_robot":
                await broadcast({
                    "type": "chat", "role": "assistant",
                    "content": "正在连接机器人，请稍候...",
                })
                loop = asyncio.get_event_loop()
                result = await loop.run_in_executor(None, skills.connect)
                await broadcast({
                    "type": "robot_status",
                    "connected": skills.connected,
                    "message": result,
                })

            elif msg_type == "disconnect_robot":
                loop = asyncio.get_event_loop()
                result = await loop.run_in_executor(None, skills.disconnect)
                await broadcast({
                    "type": "robot_status",
                    "connected": skills.connected,
                    "message": result,
                })

            elif msg_type == "stop_agent":
                agent.stop()
                await broadcast({
                    "type": "chat", "role": "assistant",
                    "content": "Agent 已收到停止指令。",
                })

            elif msg_type == "run_task":
                content = msg.get("content", "").strip()
                if content and skills.connected:
                    asyncio.create_task(agent.run_task(content))

            elif msg_type == "emergency_stop":
                skills.emergency_stop()
                agent.stop()
                await broadcast({
                    "type": "chat", "role": "assistant",
                    "content": "紧急停止已执行！",
                })

    except WebSocketDisconnect:
        pass
    except Exception as e:
        logger.error(f"WebSocket 异常: {e}")
    finally:
        connected_websockets.discard(ws)
        logger.info(f"WebSocket 已断开 (剩余 {len(connected_websockets)} 个)")


def _is_task_command(text: str) -> bool:
    """简单判断是否为任务指令（而非闲聊）"""
    task_keywords = [
        "抓", "夹", "放", "移动", "走", "转", "拿", "拾", "搬", "推", "拉",
        "打开", "关闭", "张开", "闭合", "抬", "降", "伸", "缩",
        "看", "拍照", "扫描", "巡检",
        "左臂", "右臂", "底盘", "云台", "夹爪",
        "grasp", "grab", "pick", "place", "move",
    ]
    return any(kw in text for kw in task_keywords)


# ── 状态推送 (定时) ──

async def status_pusher():
    """定时推送关节状态"""
    while True:
        if connected_websockets and skills.connected:
            try:
                positions = skills.get_all_positions()
                await broadcast({
                    "type": "joint_update",
                    "positions": positions,
                })
            except Exception:
                pass
        await asyncio.sleep(1.0)


@app.on_event("startup")
async def on_startup():
    asyncio.create_task(status_pusher())
    logger.info(f"服务已启动: http://{SERVER_HOST}:{SERVER_PORT}")


@app.on_event("shutdown")
async def on_shutdown():
    skills.disconnect()
    await llm.close()


# ── 入口 ──

def main():
    import uvicorn
    uvicorn.run(
        "loop_agent.server:app",
        host=SERVER_HOST,
        port=SERVER_PORT,
        reload=False,
        log_level="info",
    )


if __name__ == "__main__":
    main()
