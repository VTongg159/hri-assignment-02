from ur3_llm_control.student_task import OBJECT_ORDERS, assignment_for_id, student_remainder


def test_all_modulo_assignments():
    for remainder, expected in enumerate(OBJECT_ORDERS):
        result = assignment_for_id(f"230200{remainder:02d}")
        assert tuple(result.values()) == expected


def test_actual_student_id():
    assert student_remainder("23020766") == (66, 0)
    assert assignment_for_id("23020766") == {
        "zone_a": "red_cube", "zone_b": "yellow_cube", "zone_c": "blue_cube"
    }
