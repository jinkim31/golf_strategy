# temporally disable Pycharm formatter for better readability
# @formatter:off

skill_models = {
    'amateur': (
        # NAME      DIST    DEV_X       DEV_Y       IS_DIST_PROPER(d: dist to pin)
        ('DR',      210.3,  13.00,      13.00*0.7,      lambda d: 300 < d),
        ('W3',      196.6,  11.50,      11.50*0.7,      lambda d: 100 < d),
        ('W5',      178.3,  9.80,       9.80*0.7,       lambda d: 100 < d),
        ('I3',      164.6,  9.00,       9.00*0.7,       lambda d: 100 < d),
        ('I4',      155.4,  8.50,       8.50*0.7,       lambda d: 100 < d),
        ('I5',      146.3,  8.00,       8.00*0.7,       lambda d: 100 < d <= 300),
        ('I6',      137.2,  7.65,       7.65*0.7,       lambda d: 100 < d <= 300),
        ('I7',      128.0,  7.40,       7.40*0.7,       lambda d: 100 < d <= 200),
        ('I8',      118.9,  6.80,       6.80*0.7,       lambda d: 100 < d <= 200),
        ('I9',      105.2,  6.30,       6.30*0.7,       lambda d: 100 < d <= 200),
        ('PW10',    96.0,   5.80,       5.80*0.7,       lambda d: 70 < d <= 200),
        ('SW9',     80,     5.20,       5.20*0.7,       lambda d: d <= 200),
        ('SW8',     70,     4.50,       4.50*0.7,       lambda d: d <= 200),
        ('SW7',     60,     3.60,       3.60*0.7,       lambda d: d <= 200),
        ('SW6',     50,     3.00,       3.00*0.7,       lambda d: d <= 200),
        ('SW5',     40,     2.40,       2.40*0.7,       lambda d: d <= 200),
        ('SW4',     30,     2.00,       2.00*0.7,       lambda d: d <= 200),
        ('SW3',     20,     1.70,       1.70*0.7,       lambda d: d <= 200),
        ('SW2',     10,     1.30,       1.30*0.7,       lambda d: d <= 200),
        ('SW1',     5,      0,          0,          lambda d: d <= 200),
    ),

    'beginner': (
        # NAME      DIST    DEV_X       DEV_Y       IS_DIST_PROPER(d: dist to pin)
        ('DR',      182,    13.00*1.4,      13.00*0.7*1.4,      lambda d: 300 < d),
        ('W3',      166,    11.50*1.4,      11.50*0.7*1.4,      lambda d: 100 < d),
        ('W5',      155,    9.80*1.4,       9.80*0.7*1.4,       lambda d: 100 < d),
        ('I3',      146,    9.00*1.4,       9.00*0.7*1.4,       lambda d: 100 < d),
        ('I4',      137,    8.50*1.4,       8.50*0.7*1.4,       lambda d: 100 < d),
        ('I5',      128,    8.00*1.4,       8.00*0.7*1.4,       lambda d: 100 < d <= 300),
        ('I6',      119,    7.65*1.4,       7.65*0.7*1.4,       lambda d: 100 < d <= 300),
        ('I7',      110,    7.40*1.4,       7.40*0.7*1.4,       lambda d: 100 < d <= 200),
        ('I8',      101,    6.80*1.4,       6.80*0.7*1.4,       lambda d: 100 < d <= 200),
        ('I9',      87,     6.30*1.4,       6.30*0.7*1.4,       lambda d: 100 < d <= 200),
        ('PW10',    73,     5.80*1.4,       5.80*0.7*1.4,       lambda d: 70 < d <= 200),
        ('SW9',     60,     5.20*1.4,       5.20*0.7*1.4,       lambda d: d <= 200),
        ('SW8',     48,     4.50*1.4,       3.60*0.7*1.4,       lambda d: d <= 200),
        ('SW7',     41,     5.20 * 1.4,     5.20*0.7*1.4,       lambda d: d <= 200),
        ('SW6',     34,     3.00*1.4,       3.00*0.7*1.4,       lambda d: d <= 200),
        ('SW5',     27,     2.40*1.4,       2.40*0.7*1.4,       lambda d: d <= 200),
        ('SW4',     21,     2.00*1.4,       2.00*0.7*1.4,       lambda d: d <= 200),
        ('SW3',     15,     1.70*1.4,       1.70*0.7*1.4,       lambda d: d <= 200),
        ('SW2',     10,     1.30*1.4,       1.30*0.7*1.4,       lambda d: d <= 200),
        ('SW1',     5,      0,          0,          lambda d: d <= 200),
    ),
}

# @formatter:on
