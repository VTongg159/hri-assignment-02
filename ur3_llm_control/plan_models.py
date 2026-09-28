from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class PlanStep:
    skill: str
    object: Optional[str] = None
    zone: Optional[str] = None

    def display(self) -> str:
        args = [x for x in (self.object, self.zone) if x]
        return f"{self.skill}({', '.join(args)})"


@dataclass(frozen=True)
class Plan:
    steps: Tuple[PlanStep, ...]
