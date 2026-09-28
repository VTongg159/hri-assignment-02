import json
import os
import time
import urllib.error
import urllib.request


class LLMError(RuntimeError):
    pass


class NineRouterClient:
    def __init__(self, base_url=None, api_key=None, model=None, timeout=30.0):
        self.base_url = (base_url or os.getenv("NINEROUTER_BASE_URL", "")).rstrip("/")
        self.api_key = api_key or os.getenv("NINEROUTER_API_KEY", "")
        self.model = model or os.getenv("NINEROUTER_MODEL", "")
        self.timeout = timeout

    def complete(self, system_prompt: str, command: str) -> str:
        missing = [name for name, value in (("NINEROUTER_BASE_URL", self.base_url),
                                             ("NINEROUTER_API_KEY", self.api_key),
                                             ("NINEROUTER_MODEL", self.model)) if not value]
        if missing:
            raise LLMError("missing configuration: " + ", ".join(missing))
        payload = json.dumps({"model": self.model, "temperature": 0,
                              "messages": [{"role": "system", "content": system_prompt},
                                           {"role": "user", "content": command}]}).encode()
        api_root = self.base_url if self.base_url.endswith("/v1") else self.base_url + "/v1"
        request = urllib.request.Request(
            api_root + "/chat/completions", data=payload,
            headers={"Authorization": "Bearer " + self.api_key, "Content-Type": "application/json"},
            method="POST")
        max_attempts = 5
        for attempt in range(max_attempts):
            try:
                with urllib.request.urlopen(request, timeout=self.timeout) as response:
                    body = response.read().decode("utf-8")
                    content_type = response.headers.get("Content-Type", "")
                return self._decode_response(body, content_type)
            except urllib.error.HTTPError as exc:
                if exc.code not in (429, 502, 503, 504) or attempt == max_attempts - 1:
                    raise LLMError(f"9Router request failed: HTTP {exc.code}") from exc
                time.sleep(min(2 ** attempt, 8))
            except (urllib.error.URLError, KeyError, IndexError, json.JSONDecodeError) as exc:
                raise LLMError(f"9Router request failed: {exc}") from exc
        raise LLMError("9Router request failed after retries")

    @staticmethod
    def _decode_response(body: str, content_type: str = "") -> str:
        if "text/event-stream" not in content_type and not body.lstrip().startswith("data:"):
            data = json.loads(body)
            return data["choices"][0]["message"]["content"]
        chunks = []
        for line in body.splitlines():
            if not line.startswith("data:"):
                continue
            payload = line[5:].strip()
            if not payload or payload == "[DONE]":
                continue
            data = json.loads(payload)
            choice = data.get("choices", [{}])[0]
            content = choice.get("delta", {}).get("content")
            if content:
                chunks.append(content)
        if not chunks:
            raise LLMError("9Router returned an empty streamed response")
        return "".join(chunks)
