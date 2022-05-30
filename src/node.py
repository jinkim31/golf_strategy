import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
from golf_env.src import golf_env
from golf_env.src import heuristic_agent
from cv_bridge import CvBridge


class Node(object):

    def __init__(self):
        # init ros
        rospy.init_node('golf_strategy')

        # inti ros communications
        self._stroke_angles_pub = rospy.Publisher('golf_stroke_angles', std_msgs.msg.Float64MultiArray, queue_size=1)
        self._club_indexes_pub = rospy.Publisher('golf_club_indexes', std_msgs.msg.Int8MultiArray, queue_size=1)
        self._club_name_pub = rospy.Publisher('golf_club_names', std_msgs.msg.String, queue_size=1)
        self._img_pub = rospy.Publisher('golf_img', sensor_msgs.msg.Image, queue_size=1)
        self._point_sub = rospy.Subscriber('golf_point', geometry_msgs.msg.Point, self._point_callback)

        # init golf env
        self.env = golf_env.GolfEnv('straight')
        self.agent = heuristic_agent.HeuristicAgent()
        self.state = self.env.reset()

        # init cv bridge
        self.bridge = CvBridge()

        # run
        while not rospy.is_shutdown():
            self.__spin_once()
            rospy.sleep(0.1)

    def __spin_once(self):
        pass

    def _point_callback(self, msg):
        # generate an episode
        self.state = self.env.reset(initial_pos=[msg.x, msg.y])
        episode_img, stroke_angles, club_indexes, club_names = self._generate_episode()
        img_msg = self.bridge.cv2_to_imgmsg(episode_img, encoding='passthrough')

        # publish results
        self._img_pub.publish(img_msg)
        self._stroke_angles_pub.publish(std_msgs.msg.Float64MultiArray(data=stroke_angles))
        self._club_indexes_pub.publish(std_msgs.msg.Int8MultiArray(data=club_indexes))
        self._club_name_pub.publish(' '.join(club_names))

    def _generate_episode(self):
        # output arrays
        stroke_angles = []
        club_names = []
        club_indexes = []

        # generate an episode
        while True:
            # generate action
            action = self.agent.step(self.state)

            # store outputs
            stroke_angles.append(action[0])
            club_indexes.append(action[1])
            club_names.append(golf_env.GolfEnv.CLUB_INFO[action[1]][golf_env.GolfEnv.ClubInfoIndex.NAME])

            # step env
            self.state, reward, termination = self.env.step(action)
            if termination:
                break

        # generate result img
        episode_img = self.env.paint()

        return episode_img, stroke_angles, club_indexes, club_names
