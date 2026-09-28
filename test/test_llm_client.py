import json

from ur3_llm_control.llm_client import NineRouterClient


def test_decodes_json_response():
    body = json.dumps({"choices": [{"message": {"content": "{\"plan\":[]}"}}]})
    assert NineRouterClient._decode_response(body, "application/json") == '{"plan":[]}'


def test_decodes_9router_sse_response():
    body = "\n\n".join((
        'data: {"choices":[{"delta":{"role":"assistant"}}]}',
        'data: {"choices":[{"delta":{"content":"{\\\"plan\\\":"}}]}',
        'data: {"choices":[{"delta":{"content":"[]}"}}]}',
        "data: [DONE]",
    ))
    assert NineRouterClient._decode_response(body, "text/event-stream") == '{"plan":[]}'
