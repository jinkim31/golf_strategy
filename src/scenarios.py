from enum import IntEnum


class ScenarioIndex(IntEnum):
    MAP_NAME = 0
    SKILL_MODEL_NAME = 1
    WEIGHT_NAME = 2


# temporally disable Pycharm formatter for better readability
# @formatter:off

scenarios = {
    #                      MAP_NAME     SKILL_MODEL     WEIGHT
    'hwangak-beginner'  : ('hwangak',   'beginner',     'hwangak_beginner.h5'),
    'hwangak-amateur'   : ('hwangak',   'amateur',      'hwangak_amateur.h5'),
    'sejong-beginner'   : ('sejong',    'beginner',     'sejong_beginner.h5'),
    'sejong-amateur'    : ('sejong',    'amateur',      'sejong_amateur.h5'),
    'yeogang-beginner'  : ('yeogang',   'beginner',     'yeogang_beginner.h5'),
    'yeogang-amateur'   : ('yeogang',   'amateur',      'yeogang_amateur.h5'),
}

# @formatter:on
