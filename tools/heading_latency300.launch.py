"""Retired global override: explicit sensor YAML supersedes SetParameter in Humble."""


def generate_launch_description():
    raise RuntimeError(
        "Ineffective latency wrapper retired. Use SALUS_NAV_EVAL_SENSOR_PROFILE_FILE "
        "with issue328_sensors_nominal300.yaml and verify GNSS latency readback.")
