import json
import re
from typing import Any

from .plan_models import Plan, PlanStep

OBJECTS = frozenset(("red_cube", "yellow_cube", "blue_cube"))
ZONES = frozenset(("zone_a", "zone_b", "zone_c"))
SKILL_FIELDS = {
    "pick": frozenset(("skill", "object")),
    "place": frozenset(("skill", "object", "zone")),
    "home": frozenset(("skill",)),
}


class PlanValidationError(ValueError):
    pass


def _strip_single_fence(text: str) -> str:
    match = re.fullmatch(r"\s*```(?:json)?\s*\n?(.*?)\n?```\s*", text, re.DOTALL | re.IGNORECASE)
    return match.group(1) if match else text


def validate_plan(raw: str) -> Plan:
    try:
        data: Any = json.loads(_strip_single_fence(raw))
    except (json.JSONDecodeError, TypeError) as exc:
        raise PlanValidationError(f"malformed JSON: {exc}") from exc
    if not isinstance(data, dict) or set(data) != {"plan"} or not isinstance(data["plan"], list):
        raise PlanValidationError("root must contain exactly one 'plan' array")
    if not data["plan"]:
        raise PlanValidationError("plan must not be empty")
    if len(data["plan"]) > 16:
        raise PlanValidationError("plan exceeds 16-step safety limit")
    held = None
    steps = []
    for index, value in enumerate(data["plan"], 1):
        if not isinstance(value, dict) or not isinstance(value.get("skill"), str):
            raise PlanValidationError(f"step {index}: object with string 'skill' required")
        skill = value["skill"]
        if skill not in SKILL_FIELDS:
            raise PlanValidationError(f"step {index}: unknown skill '{skill}'")
        extra = set(value) - SKILL_FIELDS[skill]
        missing = SKILL_FIELDS[skill] - set(value)
        if extra:
            raise PlanValidationError(f"step {index}: unexpected field(s): {', '.join(sorted(extra))}")
        if missing:
            raise PlanValidationError(f"step {index}: missing field(s): {', '.join(sorted(missing))}")
        obj = value.get("object")
        zone = value.get("zone")
        if obj is not None and obj not in OBJECTS:
            raise PlanValidationError(f"step {index}: invalid object '{obj}'")
        if zone is not None and zone not in ZONES:
            raise PlanValidationError(f"step {index}: invalid zone '{zone}'")
        if skill == "pick":
            if held is not None:
                raise PlanValidationError(f"step {index}: cannot pick while holding '{held}'")
            held = obj
        elif skill == "place":
            if held != obj:
                raise PlanValidationError(f"step {index}: place requires corresponding pick of '{obj}'")
            held = None
        steps.append(PlanStep(skill, obj, zone))
    if held is not None:
        raise PlanValidationError(f"plan ends while holding '{held}'")
    return Plan(tuple(steps))
