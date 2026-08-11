from salus_localization.imu_normalizer import covariance_is_zero, default_covariance


def test_zero_covariance_is_detected() -> None:
    assert covariance_is_zero([0.0] * 9)
    assert not covariance_is_zero([0.0, 0.1] + [0.0] * 7)


def test_default_covariance_is_diagonal() -> None:
    assert default_covariance(0.1) == [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
