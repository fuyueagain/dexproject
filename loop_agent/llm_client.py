"""LLM 客户端 — 对接 Responses API (SSE 流式)"""

import json
import logging
from dataclasses import dataclass, field
from typing import AsyncGenerator, Optional

import httpx

from .config import LLM_API_KEY, LLM_API_URL, LLM_FALLBACK_MODEL, LLM_MAX_TOKENS, LLM_MODEL, LLM_TIMEOUT

logger = logging.getLogger("loop_agent.llm")


@dataclass
class FunctionCall:
    call_id: str = ""
    name: str = ""
    arguments: str = ""

    def parsed_arguments(self) -> dict:
        try:
            return json.loads(self.arguments)
        except (json.JSONDecodeError, TypeError):
            return {}


@dataclass
class LLMResponse:
    text: str = ""
    function_calls: list[FunctionCall] = field(default_factory=list)
    raw_output: list[dict] = field(default_factory=list)
    error: Optional[str] = None


@dataclass
class StreamEvent:
    """流式事件，推送给前端"""
    type: str  # "text_delta" | "tool_start" | "tool_delta" | "tool_done" | "done" | "error"
    content: str = ""
    tool_name: str = ""
    call_id: str = ""


class LLMClient:
    def __init__(self, api_url: str = LLM_API_URL, api_key: str = LLM_API_KEY,
                 model: str = LLM_MODEL):
        self.api_url = api_url
        self.api_key = api_key
        self.model = model
        self._client: Optional[httpx.AsyncClient] = None

    async def _get_client(self) -> httpx.AsyncClient:
        if self._client is None or self._client.is_closed:
            self._client = httpx.AsyncClient(timeout=httpx.Timeout(LLM_TIMEOUT, connect=30))
        return self._client

    async def close(self):
        if self._client and not self._client.is_closed:
            await self._client.aclose()

    async def create_response(
        self,
        input_messages: list[dict],
        tools: Optional[list[dict]] = None,
        instructions: Optional[str] = None,
    ) -> LLMResponse:
        """非流式调用，返回完整响应。

        注意: 此 API 即使 stream=false 也返回 SSE 格式，需要特殊解析。
        """
        body = {
            "model": self.model,
            "input": input_messages,
            "stream": True,  # API 始终返回 SSE，不如直接用 stream
            "max_output_tokens": LLM_MAX_TOKENS,
        }
        if tools:
            body["tools"] = tools
        if instructions:
            body["instructions"] = instructions

        result = LLMResponse()
        fc_dict: dict[str, FunctionCall] = {}

        try:
            async for event in self.create_response_stream(
                input_messages, tools, instructions
            ):
                if event.type == "text_delta":
                    result.text += event.content
                elif event.type == "tool_start":
                    fc = FunctionCall(call_id=event.call_id, name=event.tool_name)
                    fc_dict[event.call_id] = fc
                elif event.type == "tool_delta":
                    if event.call_id in fc_dict:
                        fc_dict[event.call_id].arguments += event.content
                elif event.type == "tool_done":
                    if event.call_id in fc_dict:
                        fc_dict[event.call_id].arguments = event.content
                elif event.type == "error":
                    result.error = event.content
                elif event.type == "done":
                    break

            result.function_calls = list(fc_dict.values())
        except Exception as e:
            logger.error(f"LLM 调用失败: {e}")
            result.error = str(e)

        return result

    async def create_response_stream(
        self,
        input_messages: list[dict],
        tools: Optional[list[dict]] = None,
        instructions: Optional[str] = None,
    ) -> AsyncGenerator[StreamEvent, None]:
        """流式调用，逐步 yield 事件。失败时自动降级到备用模型。"""
        body = {
            "model": self.model,
            "input": input_messages,
            "stream": True,
            "max_output_tokens": LLM_MAX_TOKENS,
        }
        if tools:
            body["tools"] = tools
        if instructions:
            body["instructions"] = instructions

        client = await self._get_client()

        try:
            async for event in self._do_stream(client, body):
                yield event
        except Exception as e:
            error_str = str(e)
            logger.error(f"LLM 流式调用失败 (model={body['model']}): {error_str}")
            if body["model"] != LLM_FALLBACK_MODEL:
                logger.info(f"降级到 {LLM_FALLBACK_MODEL} 重试...")
                body["model"] = LLM_FALLBACK_MODEL
                async for event in self._do_stream(client, body):
                    yield event
            else:
                yield StreamEvent(type="error", content=error_str)

    async def _do_stream(
        self, client: httpx.AsyncClient, body: dict
    ) -> AsyncGenerator[StreamEvent, None]:
        """内部流式请求实现"""
        accumulated_text = ""
        function_calls: dict[str, FunctionCall] = {}  # key = call_id
        id_to_call_id: dict[str, str] = {}  # item.id → call_id 映射

        try:
            async with client.stream(
                "POST",
                self.api_url,
                json=body,
                headers={
                    "Content-Type": "application/json",
                    "Authorization": f"Bearer {self.api_key}",
                },
            ) as resp:
                resp.raise_for_status()
                buffer = ""
                async for chunk in resp.aiter_text():
                    buffer += chunk
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()

                        if not line:
                            continue

                        if line.startswith("event:"):
                            continue

                        if line.startswith("data:"):
                            data_str = line[5:].strip()
                            if data_str == "[DONE]":
                                yield StreamEvent(type="done")
                                return

                            try:
                                data = json.loads(data_str)
                            except json.JSONDecodeError:
                                continue

                            event_type = data.get("type", "")

                            if event_type == "response.output_text.delta":
                                delta = data.get("delta", "")
                                accumulated_text += delta
                                yield StreamEvent(type="text_delta", content=delta)

                            elif event_type == "response.output_item.added":
                                item = data.get("item", {})
                                if item.get("type") == "function_call":
                                    call_id = item.get("call_id", "")
                                    item_id = item.get("id", "")
                                    fc = FunctionCall(
                                        call_id=call_id or item_id,
                                        name=item.get("name", ""),
                                        arguments=""
                                    )
                                    key = call_id or item_id
                                    function_calls[key] = fc
                                    if item_id and item_id != key:
                                        id_to_call_id[item_id] = key
                                    if call_id and call_id != key:
                                        id_to_call_id[call_id] = key
                                    yield StreamEvent(
                                        type="tool_start",
                                        tool_name=fc.name,
                                        call_id=key
                                    )

                            elif event_type == "response.function_call_arguments.delta":
                                fc_key = self._resolve_fc_key(
                                    data, function_calls, id_to_call_id)
                                if fc_key:
                                    delta = data.get("delta", "")
                                    function_calls[fc_key].arguments += delta
                                    yield StreamEvent(
                                        type="tool_delta",
                                        content=delta,
                                        call_id=fc_key
                                    )

                            elif event_type == "response.function_call_arguments.done":
                                fc_key = self._resolve_fc_key(
                                    data, function_calls, id_to_call_id)
                                args_str = data.get("arguments", "")
                                if fc_key:
                                    function_calls[fc_key].arguments = args_str
                                    yield StreamEvent(
                                        type="tool_done",
                                        content=args_str,
                                        tool_name=function_calls[fc_key].name,
                                        call_id=fc_key
                                    )

                            elif event_type in ("response.completed", "response.done"):
                                yield StreamEvent(type="done")
                                return

                            # 兼容 Chat Completions 格式 (delta.content / delta.tool_calls)
                            choices = data.get("choices", [])
                            if choices:
                                delta = choices[0].get("delta", {})
                                if "content" in delta and delta["content"]:
                                    yield StreamEvent(type="text_delta", content=delta["content"])
                                if "tool_calls" in delta:
                                    for tc in delta["tool_calls"]:
                                        tc_idx = str(tc.get("index", 0))
                                        if "function" in tc:
                                            fn = tc["function"]
                                            if tc.get("id"):
                                                fc = FunctionCall(
                                                    call_id=tc["id"],
                                                    name=fn.get("name", ""),
                                                    arguments=""
                                                )
                                                function_calls[tc_idx] = fc
                                                yield StreamEvent(
                                                    type="tool_start",
                                                    tool_name=fc.name,
                                                    call_id=tc["id"]
                                                )
                                            if fn.get("arguments"):
                                                if tc_idx in function_calls:
                                                    function_calls[tc_idx].arguments += fn["arguments"]
                                                    yield StreamEvent(
                                                        type="tool_delta",
                                                        content=fn["arguments"],
                                                        call_id=function_calls[tc_idx].call_id
                                                    )
                                finish_reason = choices[0].get("finish_reason")
                                if finish_reason:
                                    for fc in function_calls.values():
                                        if fc.arguments:
                                            yield StreamEvent(
                                                type="tool_done",
                                                content=fc.arguments,
                                                tool_name=fc.name,
                                                call_id=fc.call_id
                                            )
                                    yield StreamEvent(type="done")
                                    return

        except Exception as e:
            raise

    @staticmethod
    def _resolve_fc_key(data: dict, fc_map: dict, id_map: dict) -> Optional[str]:
        """从 SSE 事件中解析出 function_call 在 fc_map 中的 key"""
        for field in ("call_id", "item_id"):
            val = data.get(field, "")
            if val:
                if val in fc_map:
                    return val
                if val in id_map:
                    return id_map[val]
        # 只有一个 function_call 时直接返回
        if len(fc_map) == 1:
            return next(iter(fc_map))
        return None

    def _parse_response(self, data: dict) -> LLMResponse:
        result = LLMResponse(raw_output=data.get("output", []))
        for item in data.get("output", []):
            if item.get("type") == "message":
                for part in item.get("content", []):
                    if part.get("type") == "output_text":
                        result.text += part.get("text", "")
            elif item.get("type") == "function_call":
                result.function_calls.append(FunctionCall(
                    call_id=item.get("call_id", item.get("id", "")),
                    name=item.get("name", ""),
                    arguments=item.get("arguments", ""),
                ))
        return result

    def build_user_message(self, text: str, images_base64: Optional[list[str]] = None) -> dict:
        """构建用户消息（含可选图片）"""
        content = [{"type": "input_text", "text": text}]
        if images_base64:
            for img_b64 in images_base64:
                content.append({
                    "type": "input_image",
                    "image_url": f"data:image/jpeg;base64,{img_b64}",
                })
        return {
            "type": "message",
            "role": "user",
            "content": content,
        }

    def build_function_call_item(self, call_id: str, name: str, arguments: str) -> dict:
        return {
            "type": "function_call",
            "call_id": call_id,
            "name": name,
            "arguments": arguments,
        }

    def build_function_output(self, call_id: str, output: str) -> dict:
        return {
            "type": "function_call_output",
            "call_id": call_id,
            "output": output,
        }
