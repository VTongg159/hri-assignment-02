from dataclasses import dataclass
from typing import Callable, Dict, Optional

from .plan_models import Plan, PlanStep


@dataclass(frozen=True)
class SkillResult:
    success: bool
    status: str
    message: str = ""


class WorldState:
    def __init__(self, locations: Optional[Dict[str, str]] = None):
        self.locations = dict(locations or {"red_cube": "source_red",
                                            "yellow_cube": "source_yellow",
                                            "blue_cube": "source_blue"})
        self.held = None

    def apply(self, step: PlanStep) -> SkillResult:
        if step.skill == "home":
            return SkillResult(True, "SUCCESS")
        if step.skill == "pick":
            if self.held:
                return SkillResult(False, "ALREADY_HOLDING")
            self.held = step.object
            return SkillResult(True, "SUCCESS")
        if self.held != step.object:
            return SkillResult(False, "OBJECT_NOT_HELD")
        occupant = next((obj for obj, loc in self.locations.items()
                         if loc == step.zone and obj != step.object), None)
        if occupant:
            return SkillResult(False, "ZONE_OCCUPIED", occupant)
        self.locations[step.object] = step.zone
        self.held = None
        return SkillResult(True, "SUCCESS")


class SkillExecutor:
    def __init__(self, call_skill: Callable[[PlanStep], SkillResult]):
        self.call_skill = call_skill

    def execute(self, plan: Plan):
        results = []
        for step in plan.steps:
            result = self.call_skill(step)
            results.append((step, result))
            if not result.success:
                break
        return results
