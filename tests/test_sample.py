from integral_timber_joints import SampleClassName


def test_calculate_answer():
    sample = SampleClassName('some', 1773, ['values'])
    assert sample.calculate_answer(0) == 42
