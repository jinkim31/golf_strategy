from enum import IntEnum


class ScenarioIndex(IntEnum):
    MAP_NAME = 0
    SKILL_MODEL_NAME = 1
    WEIGHT_NAME = 2


# temporally disable Pycharm formatter for better readability
# @formatter:off

scenarios = {
    'hwangak-beginner': ('hwangak', 'beginner', 'hwangak_beginner.h5'),
    'hwangak-amateur': ('hwangak', 'amateur', 'hwangak_amateur.h5'),
    'sejong-amateur': ('sejong', 'amateur', 'sejong_amateur.h5'),
}

# @formatter:on
