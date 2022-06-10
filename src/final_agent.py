from golf_env.src import golf_env
import numpy as np


class FinalAgent:

    def __init__(self):
        pass

    def step(self, state):
        dist_to_pin = state[1]
        club_n = len(golf_env.GolfEnv.SKILL_MODEL)

        club = min(golf_env.GolfEnv.SKILL_MODEL, key=lambda c: (c[golf_env.GolfEnv.ClubInfoIndex.DIST] - dist_to_pin)**2)
        club_index = golf_env.GolfEnv.SKILL_MODEL.index(club)

        return np.random.uniform(-0, 0), club_index
