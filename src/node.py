import os
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import golf_strategy.msg
import std_msgs.msg
import rl_agent
from golf_env.src import golf_env
import final_agent
from cv_bridge import CvBridge
import numpy as np
import tensorflow as tf
import scenarios
import skill_models
import playsound
import sound_selector


class Node(object):
    # exceptions
    class NoSuchScenarioException(Exception):
        def __init__(self, name):
            self.name = name

        def __str__(self):
            return 'No such scenario name ' \
                   + str(self.name) + '. ' \
                   + 'Scenario name can be one of following: ' \
                   + str(list(scenarios.scenarios.keys()))

    # consts
    _MAX_TIMESTEP = 10
    _F_AGENT_THRESHOLD = 0.0
    _PLAY_SOUNDS = False

    def __init__(self):
        # init ros
        rospy.init_node('golf_strategy')

        # init golf variables
        self.env = None
        self.agent = None
        self.f_agent = final_agent.FinalAgent()
        self.scenario_name = ''

        # inti ros communications
        self._img_pub = rospy.Publisher('golf_img', sensor_msgs.msg.Image, queue_size=1)
        self._advice_pub = rospy.Publisher('golf_advice', golf_strategy.msg.GolfAdvice, queue_size=1)
        self._advice_text_pub = rospy.Publisher('golf_advice_text', golf_strategy.msg.GolfAdvice, queue_size=1)
        self._point_sub = rospy.Subscriber('golf_point', geometry_msgs.msg.Point, self._point_callback, queue_size=1)
        self._map_name_sub = rospy.Subscriber('golf_scenario_name', std_msgs.msg.String, self._scenario_name_callback,
                                              queue_size=1)

        # init cv bridge
        self.bridge = CvBridge()

        # run
        print('golf_strategy init done.')
        while not rospy.is_shutdown():
            self.__spin_once()
            rospy.sleep(0.1)

    def __spin_once(self):
        pass

    def _setup_scenario(self, scenario_name):
        # get scenario
        if scenario_name in scenarios.scenarios:
            scenario = scenarios.scenarios[scenario_name]
            self.scenario_name = scenario_name
        else:
            raise self.NoSuchScenarioException(scenario_name)
            return

        # init golf env
        self.env = golf_env.GolfEnv(scenario[scenarios.ScenarioIndex.MAP_NAME])
        self.env.set_skill_model(skill_models.skill_models[scenario[scenarios.ScenarioIndex.SKILL_MODEL_NAME]])
        self.state = self.env.reset(max_timestep=self._MAX_TIMESTEP)

        # init agent
        self.agent = rl_agent.SACagent()
        self.agent.load_weights(scenario[scenarios.ScenarioIndex.WEIGHT_NAME])

        print('scenario set to ' + scenario_name)

    def _scenario_name_callback(self, msg):
        # setup scenario
        self._setup_scenario(msg.data)

        # publish result
        advice_msg = golf_strategy.msg.GolfAdvice()
        img_msg = self.bridge.cv2_to_imgmsg(self.env.paint(), encoding='passthrough')
        self._img_pub.publish(img_msg)
        self._advice_pub.publish(advice_msg)

    def _point_callback(self, msg):
        # check scenario setup
        if self.scenario_name == '':
            print('Scenario not set up yet. Publish std_msgs/String /golf_scenario_name to setup scenario.')
            return

        # generate an episode
        self.state = self.env.reset(initial_pos=[msg.x, msg.y], max_timestep=self._MAX_TIMESTEP)
        successful, episode_img, advice_msg = self._generate_episode()
        img_msg = self.bridge.cv2_to_imgmsg(episode_img, encoding='passthrough')

        # publish results
        if successful:
            self._img_pub.publish(img_msg)
            self._advice_pub.publish(advice_msg)

            if self._PLAY_SOUNDS:
                club_sound_path = os.path.join(
                    os.path.dirname(__file__),
                    '../sounds',
                    sound_selector.club_to_sound[advice_msg.club_names[0]]
                )
                playsound.playsound(club_sound_path)

                angle_sound_path = os.path.join(
                    os.path.dirname(__file__),
                    '../sounds',
                    sound_selector.angle_to_sound(advice_msg.stroke_angles[0])
                )
                playsound.playsound(angle_sound_path)

        else:
            print('timestep exceeded max_timestep.')

    def _generate_episode(self):
        # make msg
        advice_msg = golf_strategy.msg.GolfAdvice()
        advice_msg.advice_text = ''
        advice_msg.advice_text_brief = ''

        # generate an episode
        while True:
            # generate action
            if self.state[1] > self._F_AGENT_THRESHOLD:
                state_img, state_dist = self.state[0], self.state[1]
                state_img = state_img.astype(np.float32) / 100.0
                state_img = np.stack((state_img, state_img, state_img), axis=2)
                state_dist = np.array(state_dist.reshape(1, ))
                mu, _, ac_d = self.agent.actor(
                    tf.convert_to_tensor([state_img], dtype=tf.float32),
                    tf.convert_to_tensor([state_dist], dtype=tf.float32)
                )
                action_c = mu.numpy()[0]
                action_d = np.argmax(ac_d)
                club_index = action_d
                stroke_angle = action_c[action_d]
            else:
                f_action = self.f_agent.step(self.state)
                stroke_angle = f_action[0]
                club_index = f_action[1]

            # store outputs
            advice_msg.nap_name = scenarios.scenarios[self.scenario_name][scenarios.ScenarioIndex.MAP_NAME]
            advice_msg.skill_name = scenarios.scenarios[self.scenario_name][scenarios.ScenarioIndex.SKILL_MODEL_NAME]
            advice_msg.stroke_angles.append(stroke_angle)
            advice_msg.club_indexes.append(club_index)
            advice_msg.club_names.append(golf_env.GolfEnv.SKILL_MODEL[club_index][golf_env.GolfEnv.ClubInfoIndex.NAME])

            # step env
            self.state, reward, termination = self.env.step(
                (stroke_angle, club_index),
                accurate_shots=True,
                debug=True
            )

            # store advice texts
            debug_str = self.env.get_state_metadata()['debug_str']
            advice_msg.advice_text += debug_str + '\n'
            advice_msg.advice_text_brief += debug_str[7:14] + debug_str[17:27] + debug_str[28:38] + '\n'

            if termination:
                break

        self.env.plot()
        # store green_strokes
        advice_msg.green_strokes = -reward - 1

        # generate result img
        episode_img = self.env.paint()

        # check timestep
        successful = True
        if self.env.get_timestep() == self._MAX_TIMESTEP:
            successful = False

        return successful, episode_img, advice_msg
