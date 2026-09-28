import pytest

from ur3_llm_control.task_validator import PlanValidationError, validate_plan


VALID = '{"plan":[{"skill":"pick","object":"red_cube"},{"skill":"place","object":"red_cube","zone":"zone_b"},{"skill":"home"}]}'


def test_valid_plan_and_fenced_json():
    assert len(validate_plan(VALID).steps) == 3
    assert len(validate_plan("```json\n" + VALID + "\n```").steps) == 3


@pytest.mark.parametrize("raw, reason", [
    ('{"plan":[{"skill":"system","command":"rm -rf /"}]}', "unknown skill 'system'"),
    ('{"plan":[{"skill":"dance"}]}', "unknown skill"),
    ('{"plan":[{"skill":"pick","object":"green_cube"}]}', "invalid object"),
    ('{"plan":[{"skill":"pick","object":"red_cube"},{"skill":"place","object":"red_cube","zone":"zone_d"}]}', "invalid zone"),
    ('not json', "malformed JSON"),
    ('{"plan":[{"skill":"home","command":"rm -rf /"}]}', "unexpected field"),
    ('{"plan":[{"skill":"move_joint","joint":2,"angle":1.5}]}', "unknown skill"),
    ('{"plan":[{"skill":"place","object":"red_cube","zone":"zone_a"}]}', "corresponding pick"),
])
def test_rejects_unsafe_plans(raw, reason):
    with pytest.raises(PlanValidationError, match=reason):
        validate_plan(raw)
