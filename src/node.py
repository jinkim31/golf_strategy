import rospy
import geometry_msgs.msg
import sensor_msgs.msg
from golf_env.src import golf_env
from golf_env.src import heuristic_agent
from cv_bridge import CvBridge


class Node(object):

    def __init__(self):
        rospy.init_node('golf_strategy')

        self._img_pub = rospy.Publisher('img', sensor_msgs.msg.Image, queue_size=10)
        self._point_sub = rospy.Subscriber('point', geometry_msgs.msg.Point, self._point_callback)
        self._count = 0
        self.bridge = CvBridge()
        self.env = golf_env.GolfEnv('sophia_green')
        self.agent = heuristic_agent.HeuristicAgent()
        self.state = self.env.reset()

        while not rospy.is_shutdown():
            self.__spin_once()
            rospy.sleep(0.5)

    def __spin_once(self):
        pass

    def _point_callback(self, msg):
        self.state = self.env.reset(initial_pos=[msg.x, msg.y])

        while True:
            self.state, reward, termination = self.env.step(self.agent.step(self.state))
            if termination:
                break

        img = self.env.paint()
        msg = self.bridge.cv2_to_imgmsg(img, encoding='passthrough')
        self._img_pub.publish(msg)
