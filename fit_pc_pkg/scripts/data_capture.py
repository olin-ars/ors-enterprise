import time
import rospy
from std_msgs.msg import String

class DataCapturer():
    def __init__(self):
        rospy.init_node('data_capturer')

        self.airmar_capture = open("AirmarData.txt", "w")
        self.hemisphere_capture = open("HemisphereData.txt", "w")

        airmar_pub = rospy.Subscriber("airmar/fullmsg", String, self.airmar_callback)
        hemisphere_pub = rospy.Subscriber("hemisphere/fullmsg", String, self.hemisphere_callback)

    def airmar_callback(self, data):
        self.airmar_capture.write(time.time() + " -- " + data)

    def hemisphere_callback(self, data):
        self.hemisphere_capture.write(time.time() + " -- " + data)

if __name__ == '__main__':
    data_capture = DataCapturer()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        print "Running..."
        r.sleep()