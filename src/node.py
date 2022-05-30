import rospy
from std_msgs.msg import String
from golf_env.src import golf_env
from golf_env.src import heuristic_agent


class Node(object):

    def __init__(self):
        self._pub = rospy.Publisher('foo', String, queue_size=10)
        self._sub = rospy.Subscriber('foo', String, self.__callback)
        self._count = 0

        self.env = golf_env.GolfEnv('straight')
        self.agent = heuristic_agent.HeuristicAgent()
        self.state = self.env.reset()

        rospy.init_node('boilerplate')

        while not rospy.is_shutdown():
            self.__spin_once()
            rospy.sleep(0.5)

    def __spin_once(self):
        self.state, reward, termination = self.env.step(self.agent.step(self.state))
        self.env.plot()
        self._count += 1
        self._pub.publish('Hello world #{}'.format(self._count))

    def __callback(self, msg):
        rospy.loginfo('Received {}'.format(msg.data))
