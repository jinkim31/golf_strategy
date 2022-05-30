import rospy
from std_msgs.msg import String
import sensor_msgs.msg
from golf_env.src import golf_env
from golf_env.src import heuristic_agent
from cv_bridge import CvBridge

class Node(object):

    def __init__(self):
        self._img_pub = rospy.Publisher('img', sensor_msgs.msg.Image, queue_size=10)
        self._sub = rospy.Subscriber('foo', String, self.__callback)
        self._count = 0
        self.bridge = CvBridge()

        self.env = golf_env.GolfEnv('sophia_green')
        self.agent = heuristic_agent.HeuristicAgent()
        self.state = self.env.reset()

        rospy.init_node('golf_strategy')

        while not rospy.is_shutdown():
            self.__spin_once()
            rospy.sleep(0.5)

    def __spin_once(self):
        self.state, reward, termination = self.env.step(self.agent.step(self.state))
        img = self.env.paint()
        msg = self.bridge.cv2_to_imgmsg(img, encoding='passthrough')
        self._img_pub.publish(msg)

    def __callback(self, msg):
        rospy.loginfo('Received {}'.format(msg.data))
