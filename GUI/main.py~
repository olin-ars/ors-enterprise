#!/usr/bin/env python
from PyQt4 import uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from functools import partial

import sys
import rospy
from math import *
from std_msgs.msg import * #for convenience
from mainUI import *
PI = 3.14159265358979

class ORSWindow(QMainWindow):
    updated = pyqtSignal()
    def __init__(self,parent=None):
        QMainWindow.__init__(self,parent)
        self.ui =  Ui_ORS_GUI()
        self.ui.setupUi(self)
        rospy.init_node('ORS_GUI',anonymous=True)
        self.sub = {}
        self.val = {}
        self.updated.connect(self.update)

        msgs = ['rudder/pos','rudder/set_point','sail/pos','sail/set_point','GPS/latitude','GPS/longitude', 'GPS/direction']
        for msg in msgs:
            if msg != 'sail/pos' and msg != 'sail/set_point':
                self.sub[msg] = rospy.Subscriber(msg,Int16,partial(self.fetchData,msg_type=msg))
                self.val[msg] = 0
            else:
                self.sub[msg] = rospy.Subscriber(msg,Float32,partial(self.fetchData,msg_type=msg))
                self.val[msg] = 0.0
        #self.sub['rPos'] = rospy.Subscriber('rudder/pos',Int16,partial(self.fetchData,msg_type='rudder/pos'))
        #self.sub['rDst'] = rospy.Subscriber('rudder/set_point',Int16,partial(self.fetchData,msg_type='rudder/set_point'))
        #self.sub['sPos'] = rospy.Subscriber('sail/pos',Int16,partial(self.fetchData,msg_type='sail/pos'))
        #self.sub['sDst'] = rospy.Subscriber('sail/set_point',Int16,partial(self.fetchData,msg_type='sail/set_point'))

    def fetchData(self,msg,msg_type=''):
        self.val[msg_type] = msg.data
        self.ui.compass.setValue(self.val['GPS/direction']*36)
        self.ui.map.setLoc(self.val['GPS/latitude']-5,self.val['GPS/longitude']-5)
        self.ui.rPosDial.setValue(self.val['rudder/pos']+180) #to have 0=North
        self.ui.rPosLabel.setText(QString.number(self.val['rudder/pos']))
        self.ui.sPosSlider.setValue(self.val['sail/pos'])
        self.ui.sPosLabel.setText(QString.number(self.val['sail/pos']))
        self.updated.emit()
    def update(self):
        QMainWindow.update(self)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = ORSWindow()
    w.show()
    sys.exit(app.exec_())
