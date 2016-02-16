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

class ORSWindow(QMainWindow):
    updated = pyqtSignal()
    def __init__(self,parent=None):
        QMainWindow.__init__(self,parent)
        self.ui =  Ui_ORS_GUI()
        self.ui.setupUi(self)
        rospy.init_node('ORS_GUI',anonymous=True)
        self.sub = {}
        self.updated.connect(self.update)
        self.ui.pushButton.clicked.connect(self.onButtonPushed)


        msgs = {'rudder/pos' : Int16,
                'rudder/set_point' : Int16,
                'sail/pos' : Float32,
                'sail/set_point' : Float32,
                'hemisphere/position' : Pose2D,
                #'hemisphere/speed' : Float32,
                #'hemisphere/track' : Float32,
                #'hemisphere/attitude' : Vector3,
                #'hemisphere/magnetic_direction' : Float32,
                #'hemisphere/status' : Bool,
                #'hemisphere/errors' : String,
                #'hemisphere/fullmsg' : String,
            }
        for msg in msgs:
            self.sub[msg] = rospy.Subscriber(msg,msgs[msg],partial(self.fetchData,msg_type=msg))
        #self.sub['rPos'] = rospy.Subscriber('rudder/pos',Int16,partial(self.fetchData,msg_type='rudder/pos'))
        #self.sub['rDst'] = rospy.Subscriber('rudder/set_point',Int16,partial(self.fetchData,msg_type='rudder/set_point'))
        #self.sub['sPos'] = rospy.Subscriber('sail/pos',Int16,partial(self.fetchData,msg_type='sail/pos'))
        #self.sub['sDst'] = rospy.Subscriber('sail/set_point',Int16,partial(self.fetchData,msg_type='sail/set_point'))

    def fetchData(self,msg,msg_type=''):
        if msg_type == 'rudder/pos':
            self.ui.rPosDial.setValue(msg.data+180) #to have 0=North
            self.ui.rPosLabel.setText(str(msg.data))

        if msg_type == 'rudder/set_point':
            self.ui.rDstLabel.setText(str(msg.data))
            
        elif msg_type == 'sail/pos':
            self.ui.sPosSlider.setValue(msg.data)
            self.ui.sPosLabel.setText(str(msg.data))

        if msg_type == 'sail/set_point':
            self.ui.sDstLabel.setText(str(msg.data))
            
        elif msg_type == 'hemisphere/position':
            self.ui.map.setLoc(msg.x*1.0/180, msg.y*1.0/180)
            self.ui.compass.setValue(msg.theta)

            self.ui.latiVal.setText(str(msg.x))
            self.ui.longVal.setText(str(msg.y))


        #elif msg_type == 'hemisphere/speed':
            
        #elif msg_type == 'hemisphere/attitude':
            
        #elif msg_type == 'hemisphere/track':

        else:
            print "Unknown message recieved: {}".format(msg_type)
            # Message recieved not currently handled

        #self.ui.map.setLoc(self.val['GPS/latitude']-5,self.val['GPS/longitude']-5)
        self.updated.emit()

    def onButtonPushed(self):
        import webbrowser #Don't do this.
        webbrowser.open("http://giphy.com/embed/UgAvyUi9mXBiE")

    def update(self):
        QMainWindow.update(self)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = ORSWindow()
    w.show()
    sys.exit(app.exec_())
