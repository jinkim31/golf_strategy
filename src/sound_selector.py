import numpy as np

club_to_sound = {
    'DR': 'dr.mp3',
    'W3': 'w3.mp3',
    'W5': 'w5.mp3',
    'I3': 'i3.mp3',
    'I4': 'i4.mp3',
    'I5': 'i5.mp3',
    'I6': 'i6.mp3',
    'I7': 'i7.mp3',
    'I8': 'i8.mp3',
    'I9': 'i9.mp3',
    'PW10': 'pw.mp3',
    'SW9': 'sw.mp3',
    'SW8': 'sw.mp3',
    'SW7': 'sw.mp3',
    'SW6': 'sw.mp3',
    'SW5': 'sw.mp3',
    'SW4': 'sw.mp3',
    'SW3': 'sw.mp3',
    'SW2': 'sw.mp3',
    'SW1': 'sw.mp3',
}


def angle_to_sound(angle):
    angles = np.linspace(-35, 35, 70 + 1)

    result = min(angles, key=lambda a: (angle - a)**2)

    if result < 0:
        result = '-' + str(int(np.abs(result)))
    else:
        result = str(int(np.abs(result)))

    result = result + '.mp3'
    print(result)
    return result
