import re
from typing import Dict

OBJECT_ORDERS = (
    ("red_cube", "yellow_cube", "blue_cube"),
    ("red_cube", "blue_cube", "yellow_cube"),
    ("yellow_cube", "red_cube", "blue_cube"),
    ("yellow_cube", "blue_cube", "red_cube"),
    ("blue_cube", "red_cube", "yellow_cube"),
    ("blue_cube", "yellow_cube", "red_cube"),
)


def assignment_for_id(student_id: str) -> Dict[str, str]:
    digits = re.sub(r"\D", "", str(student_id))
    if len(digits) < 2:
        raise ValueError("student_id must contain at least two digits")
    order = OBJECT_ORDERS[int(digits[-2:]) % 6]
    return dict(zip(("zone_a", "zone_b", "zone_c"), order))


def student_remainder(student_id: str) -> tuple[int, int]:
    digits = re.sub(r"\D", "", str(student_id))
    if len(digits) < 2:
        raise ValueError("student_id must contain at least two digits")
    xx = int(digits[-2:])
    return xx, xx % 6
