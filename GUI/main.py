#!/usr/bin/env python
from PyQt4 import uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *

import sys
import rospy
from math import *
from std_msgs.msg import * #for convenience
from mainUI import *
PI = 3.14159265358979

class ORSWindow(QMainWindow):
    def __init__(self,parent=None):
        QMainWindow.__init__(self,parent);
        self.ui =  Ui_ORS_GUI();
        self.ui.setupUi(self);
        rospy.init_node('ORS_GUI',anonymous=True);
        self.RCRSub = rospy.Subscriber('sample_msg',Float32,self.fetchData);
        self.theta = 0;
    def fetchData(self,msg):
        print(msg.data);
        self.theta = msg.data;
        self.update();
    def update(self):
        self.ui.compass.setValue(self.theta * 180 / PI);
        self.ui.map.setLoc(self.theta,self.theta);
        QMainWindow.update(self);

if __name__ == "__main__":
    app = QApplication(sys.argv);
    w = ORSWindow();
    w.setWindowTitle("ORSVisualize");
    w.show();
    sys.exit(app.exec_());
