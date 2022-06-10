# temporally disable Pycharm formatter for better readability
# @formatter:off

skill_models = {
    'amateur': (
        # NAME      DIST    DEV_X       DEV_Y       IS_DIST_PROPER(d: dist to pin)
        ('DR',      210.3,  54 / 6,   8.6 / 3,    lambda d: 300 < d),
        ('W3',      196.6,  50 / 6,   7.6 / 3,    lambda d: 100 < d),
        ('W5',      178.3,  36 / 6,   6.6 / 3,    lambda d: 100 < d),
        ('I3',      164.6,  36 / 6,   5.9 / 3,    lambda d: 100 < d),
        ('I4',      155.4,  32.0 / 6,   5.5 / 3,    lambda d: 100 < d),
        ('I5',      146.3,  27.4 / 6,   5.1 / 3,    lambda d: 100 < d <= 300),
        ('I6',      137.2,  27.4 / 6,   4.8 / 3,    lambda d: 100 < d <= 300),
        ('I7',      128.0,  27.4 / 6,   4.5 / 3,    lambda d: 100 < d <= 200),
        ('I8',      118.9,  27.4 / 6,   4.3 / 3,    lambda d: 100 < d <= 200),
        ('I9',      105.2,  32.0 / 6,   3.9 / 3,    lambda d: 100 < d <= 200),
        ('PW10',    96.0,   36.6 / 6,   3.7 / 3,    lambda d: 70 < d <= 200),
        ('SW9',     80,     36.6 / 6,   3.5 / 3,    lambda d: d <= 200),
        ('SW8',     70,     32.0 / 6,   3.2 / 3,    lambda d: d <= 200),
        ('SW7',     60,     30 / 6,     3.1 / 3,    lambda d: d <= 200),
        ('SW6',     50,     20 / 6,     2.7 / 3,    lambda d: d <= 200),
        ('SW5',     40,     15 / 6,     2.3 / 3,    lambda d: d <= 200),
        ('SW4',     30,     10 / 6,     2.0 / 3,    lambda d: d <= 200),
        ('SW3',     20,     5 / 6,      1.0 / 3,    lambda d: d <= 200),
        ('SW2',     10,     3 / 6,      0.5 / 3,    lambda d: d <= 200),
        ('SW1',     5,      1 / 6,      0,    lambda d: d <= 200),
    ),

    'beginner': (
        # NAME      DIST    DEV_X       DEV_Y       IS_DIST_PROPER(d: dist to pin)
        ('DR',      182,    54 / 5,     8.6 / 2,    lambda d: 300 < d),
        ('W3',      165,    50 / 5,     7.6 / 2,    lambda d: 100 < d),
        ('W5',      155,    36 / 5,     6.6 / 2,    lambda d: 100 < d),
        ('I3',      146,    36 / 5,     5.9 / 2,    lambda d: 100 < d),
        ('I4',      137,    32.0 / 5,   5.5 / 2,    lambda d: 100 < d),
        ('I5',      128,    27.4 / 5,   5.1 / 2,    lambda d: 100 < d <= 300),
        ('I6',      119,    27.4 / 5,   4.8 / 2,    lambda d: 100 < d <= 300),
        ('I7',      110,    27.4 / 5,   4.5 / 2,    lambda d: 100 < d <= 200),
        ('I8',      101,    27.4 / 5,   4.3 / 2,    lambda d: 100 < d <= 200),
        ('I9',      87,     32.0 / 5,   3.9 / 2,    lambda d: 100 < d <= 200),
        ('PW10',    73,     36.6 / 5,   3.7 / 2,    lambda d: 70 < d <= 200),
        ('SW9',     60,     36.6 / 5,   3.3 / 2,    lambda d: d <= 200),
        ('SW8',     48,     32.0 / 5,   3.2 / 2,    lambda d: d <= 200),
        ('SW7',     41,     30 / 5,     3.1 / 2,    lambda d: d <= 200),
        ('SW6',     34,     20 / 5,     3.0 / 2,    lambda d: d <= 200),
        ('SW5',     27,     15 / 5,     2.9 / 2,    lambda d: d <= 200),
        ('SW4',     21,     10 / 5,     2.8 / 2,    lambda d: d <= 200),
        ('SW3',     15,     5 / 5,      2.7 / 2,    lambda d: d <= 200),
        ('SW2',     10,     3 / 5,      2.7 / 2,    lambda d: d <= 200),
        ('SW1',     5,      0,          0,          lambda d: d <= 200),
    )
}

# @formatter:on
