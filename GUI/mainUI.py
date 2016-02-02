# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main.ui'
#
# Created: Tue Feb  2 02:51:54 2016
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_ORS_GUI(object):
    def setupUi(self, ORS_GUI):
        ORS_GUI.setObjectName(_fromUtf8("ORS_GUI"))
        ORS_GUI.resize(800, 600)
        self.centralwidget = QtGui.QWidget(ORS_GUI)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.pushButton = QtGui.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(630, 320, 111, 61))
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.gridLayoutWidget = QtGui.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(500, 30, 261, 221))
        self.gridLayoutWidget.setObjectName(_fromUtf8("gridLayoutWidget"))
        self.gridLayout = QtGui.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setMargin(0)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.map = MapWidget(self.gridLayoutWidget)
        self.map.setFrameShape(QtGui.QFrame.StyledPanel)
        self.map.setFrameShadow(QtGui.QFrame.Raised)
        self.map.setObjectName(_fromUtf8("map"))
        self.gridLayout.addWidget(self.map, 0, 0, 1, 1)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label = QtGui.QLabel(self.gridLayoutWidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.horizontalLayout_2.addWidget(self.label)
        self.latiVal = QtGui.QLabel(self.gridLayoutWidget)
        self.latiVal.setObjectName(_fromUtf8("latiVal"))
        self.horizontalLayout_2.addWidget(self.latiVal)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.longLab = QtGui.QLabel(self.gridLayoutWidget)
        self.longLab.setObjectName(_fromUtf8("longLab"))
        self.horizontalLayout_3.addWidget(self.longLab)
        self.longVal = QtGui.QLabel(self.gridLayoutWidget)
        self.longVal.setObjectName(_fromUtf8("longVal"))
        self.horizontalLayout_3.addWidget(self.longVal)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.gridLayout.addLayout(self.verticalLayout, 1, 0, 1, 1)
        self.gridLayout_2 = QtGui.QGridLayout()
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.label_2 = QtGui.QLabel(self.gridLayoutWidget)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.gridLayout_2.addWidget(self.label_2, 0, 0, 1, 1)
        self.label_4 = QtGui.QLabel(self.gridLayoutWidget)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.gridLayout_2.addWidget(self.label_4, 1, 0, 1, 1)
        self.label_5 = QtGui.QLabel(self.gridLayoutWidget)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.gridLayout_2.addWidget(self.label_5, 1, 1, 1, 1)
        self.label_3 = QtGui.QLabel(self.gridLayoutWidget)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.gridLayout_2.addWidget(self.label_3, 0, 1, 1, 1)
        self.gridLayout.addLayout(self.gridLayout_2, 1, 1, 1, 1)
        self.compass = CompassWidget(self.gridLayoutWidget)
        self.compass.setObjectName(_fromUtf8("compass"))
        self.gridLayout.addWidget(self.compass, 0, 1, 1, 1)
        ORS_GUI.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(ORS_GUI)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        ORS_GUI.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(ORS_GUI)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        ORS_GUI.setStatusBar(self.statusbar)

        self.retranslateUi(ORS_GUI)
        QtCore.QMetaObject.connectSlotsByName(ORS_GUI)

    def retranslateUi(self, ORS_GUI):
        ORS_GUI.setWindowTitle(_translate("ORS_GUI", "ORS_GUI", None))
        self.pushButton.setText(_translate("ORS_GUI", "Launch Boat", None))
        self.label.setText(_translate("ORS_GUI", "Latitude :", None))
        self.latiVal.setText(_translate("ORS_GUI", "--", None))
        self.longLab.setText(_translate("ORS_GUI", "Longitude :", None))
        self.longVal.setText(_translate("ORS_GUI", "--", None))
        self.label_2.setText(_translate("ORS_GUI", "Direction:", None))
        self.label_4.setToolTip(_translate("ORS_GUI", "Measured From North, Clockwise", None))
        self.label_4.setText(_translate("ORS_GUI", "Angle", None))
        self.label_5.setText(_translate("ORS_GUI", "--", None))
        self.label_3.setText(_translate("ORS_GUI", "--", None))

from CompassWidget import CompassWidget
from MapWidget import MapWidget
