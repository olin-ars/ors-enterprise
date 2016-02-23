#!/usr/bin/env python
from PyQt4 import uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from functools import partial

import sys
import rospy
from math import *
from std_msgs.msg import * #for convenience
from geometry_msgs.msg import Pose2D, Vector3
from mainUI import *

#torpedo
import webbrowser
class IgnoreMouseFilter(QObject):
    def eventFilter(self,obj,event):
        if event.type() == QEvent.KeyPress:
            print("Key Pressed")
            return True
        return False

class ORSWindow(QMainWindow):
    updated = pyqtSignal()
    def __init__(self,parent=None):
        #INITIALIZE UI
        QMainWindow.__init__(self,parent)
        self.ui =  Ui_ORS_GUI()
        self.ui.setupUi(self)
        rospy.init_node('ORS_GUI',anonymous=True)
        self.mode_rBtn = [
                self.ui.mode_dfRBtn,
                self.ui.mode_rcRBtn,
                self.ui.mode_autoRBtn,
                self.ui.mode_testRBtn,
                ]
        self.installEventFilter(IgnoreMouseFilter(self))
        
        #for w in vars(self.ui):
        #    print(w)
        #    try:
        #        w.setEnabled(False)
        #    except:
        #        pass
        
        #SUBSCRIBE TO ROS TOPICS
        self.subscribe()
        
        #USER-EVENTS
        self.updated.connect(self.update)
        self.ui.torpedoBtn.clicked.connect(self.onTorpedo)
        self.ui.launchBoatBtn.clicked.connect(self.onLaunchBoat)
        self.ui.publishBtn.clicked.connect(self.onPublish)

        #publishing to rostopics
        self.proc = QProcess()

    def subscribe(self):
        self.sub = {}
        msgs = {'rudder/pos' : Int16,
                'rudder/set_point' : Int16,
                'sail/pos' : Float32,
                'sail/set_point' : Float32,
                'hemisphere/position' : Pose2D,
                'hemisphere/speed' : Float32,
                'hemisphere/track' : Float32,
                'hemisphere/attitude' : Vector3,
                #'hemisphere/magnetic_direction' : Float32,
                #'hemisphere/status' : Bool,
                #'hemisphere/errors' : String,
                #'hemisphere/fullmsg' : String,
                'operating_mode' : Int16,
            }
        for msg in msgs:
            self.sub[msg] = rospy.Subscriber(msg,msgs[msg],partial(self.fetchData,msg_type=msg))

    def fetchData(self,msg,msg_type=''):

        # RUDDER/SAIL
        if msg_type == 'rudder/pos':
            self.ui.rPosDial.setValue(msg.data+180) #to have 0=North
            self.ui.rPosLabel.setText(str(msg.data))

        elif msg_type == 'rudder/set_point':
            self.ui.rDstLabel.setText(str(msg.data))
            
        elif msg_type == 'sail/pos':
            self.ui.sPosSlider.setValue(msg.data)
            self.ui.sPosLabel.setText(str(msg.data))

        elif msg_type == 'sail/set_point':
            self.ui.sDstLabel.setText(str(msg.data))
        #HEMISPHERE
        elif msg_type == 'hemisphere/position':
            self.ui.map.setLoc(msg.x*1.0/180, msg.y*1.0/180)
            self.ui.compass.setValue(msg.theta)
            self.ui.latiLabel.setText(str(msg.x))
            self.ui.longLabel.setText(str(msg.y))
        
        elif msg_type == 'hemisphere/speed':
            self.ui.h_spdLabel.setText(str(msg.data))
            
        elif msg_type == 'hemisphere/attitude':
            self.ui.h_attXLabel.setText(str(msg.x))
            self.ui.h_attYLabel.setText(str(msg.y))
            self.ui.h_attZLabel.setText(str(msg.z))
        
        elif msg_type == 'hemisphere/track':
            self.ui.h_trkLabel.setText(str(msg.data))
        elif msg_type == 'operating_mode':
            print("OPMODE")
            DEFAULT=0
            RC_MODE=1
            AUTO_MODE=2
            TEST_mode=3
            for btn in self.mode_rBtn:
                btn.setChecked(False)

            if msg.data == DEFAULT:
                self.ui.mode_dfRBtn.setChecked(True);
            elif msg.data == RC_MODE:
                self.ui.mode_rcRBtn.setChecked(True);
            elif msg.data == AUTO_MODE:
                self.ui.mode_autoRBtn.setChecked(True);
            elif msg_data == TEST_MODE:
                self.ui.mode_testRBtn.setChecked(True);
        else:
            print "Unknown message recieved: {}".format(msg_type)
            # Message received not currently handled

        #self.ui.map.setLoc(self.val['GPS/latitude']-5,self.val['GPS/longitude']-5)
        self.updated.emit()

    def onTorpedo(self):
        webbrowser.open("http://giphy.com/embed/UgAvyUi9mXBiE")

    def onLaunchBoat(self):
        QProcess.startDetached("roslaunch fit_pc_pkg RC_code.launch")

    def onPublish(self):
        try:
            self.proc.close()
        except:
            pass
        
        topic_name = self.ui.topicNameEdit.text() 
        data_type = self.ui.dataTypeEdit.text()
        data_value = self.ui.dataValueEdit.text()
        
        cmd = 'rostopic pub' + ' '
        cmd += topic_name + ' '
        cmd += data_type + ' '
        cmd += '"' + data_value + '"'

        self.proc.start(cmd)

    def update(self):
        QMainWindow.update(self)
    def mousePressEvent(self,event):
        print(event)
        event.ignore()
        return False

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = ORSWindow()
    w.show()
    sys.exit(app.exec_())
