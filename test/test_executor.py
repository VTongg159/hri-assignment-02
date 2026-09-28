from ur3_llm_control.plan_models import PlanStep
from ur3_llm_control.skill_executor import WorldState


def test_pick_place_state_transition():
    world = WorldState()
    assert world.apply(PlanStep("pick", "red_cube")).success
    assert world.apply(PlanStep("place", "red_cube", "zone_b")).success
    assert world.locations["red_cube"] == "zone_b"


def test_place_without_pick_and_occupied_zone():
    world = WorldState({"red_cube": "zone_a", "yellow_cube": "source_yellow"})
    assert world.apply(PlanStep("place", "yellow_cube", "zone_a")).status == "OBJECT_NOT_HELD"
    assert world.apply(PlanStep("pick", "yellow_cube")).success
    assert world.apply(PlanStep("place", "yellow_cube", "zone_a")).status == "ZONE_OCCUPIED"


def test_buffer_breaks_three_object_cycle():
    world = WorldState({"red_cube": "zone_a", "blue_cube": "zone_b", "yellow_cube": "zone_c"})
    # Deterministic internal buffer sequence for A:red,B:blue,C:yellow -> A:blue,B:yellow,C:red.
    sequence = (("red_cube", "buffer_zone"), ("blue_cube", "zone_a"),
                ("yellow_cube", "zone_b"), ("red_cube", "zone_c"))
    for obj, target in sequence:
        assert world.apply(PlanStep("pick", obj)).success
        assert world.apply(PlanStep("place", obj, target)).success
    assert world.locations == {"red_cube": "zone_c", "blue_cube": "zone_a", "yellow_cube": "zone_b"}
