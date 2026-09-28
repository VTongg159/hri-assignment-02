import json
from pathlib import Path

from .llm_client import NineRouterClient


class Planner:
    def __init__(self, mode: str, prompt_path: str, assignment):
        self.mode = mode
        self.prompt = Path(prompt_path).read_text(encoding="utf-8")
        self.assignment = assignment

    def plan(self, command: str) -> str:
        context = self.prompt + "\nPersonalized assignment:\n" + json.dumps(self.assignment)
        if self.mode == "mock":
            return self._mock(command)
        return NineRouterClient().complete(context, command)

    def _mock(self, command: str) -> str:
        lower = command.casefold()
        advanced = "student id" in lower or "mã sinh viên" in lower or "according" in lower
        if advanced:
            steps = []
            for zone, obj in self.assignment.items():
                steps += [{"skill": "pick", "object": obj},
                          {"skill": "place", "object": obj, "zone": zone}]
            steps.append({"skill": "home"})
            return json.dumps({"plan": steps})
        objects = {"red_cube": ("red", "đỏ"), "yellow_cube": ("yellow", "vàng"),
                   "blue_cube": ("blue", "xanh")}
        zones = {"zone_a": ("zone a", "vùng a", "ô a"),
                 "zone_b": ("zone b", "vùng b", "ô b"),
                 "zone_c": ("zone c", "vùng c", "ô c")}
        obj = next((key for key, words in objects.items() if any(x in lower for x in words)), None)
        zone = next((key for key, words in zones.items() if any(x in lower for x in words)), None)
        if not obj or not zone:
            return json.dumps({"plan": [{"skill": "unsupported_request"}]})
        return json.dumps({"plan": [{"skill": "pick", "object": obj},
                                    {"skill": "place", "object": obj, "zone": zone},
                                    {"skill": "home"}]})
