import sys

from PyQt4.QtGui import *
from PyQt4.QtCore import *

class MapWidget(QFrame):
    def __init__(self,parent=None):
        QFrame.__init__(self,parent);
        self.location = QPoint(0,0);
    def setLoc(self,*args):
        """setLoc : set latitude/longitude
        setLoc(int latitude, int longitude);
        setLoc(QPoint)
        setLoc(QPointF)
        """

    def paintEvent(self,event):
        p = QPainter(self);
        p.scale(self.width()/2,self.height()/2); #viewport from -1~1
        p.translate(1,1);
        m = QPixmap('world.gif');
        p.drawPixmap(QRectF(-1,-1,2,2),m,QRectF(0,0,m.width(),m.height()));
        QFrame.paintEvent(self,event);

if __name__ == "__main__":
    global app;
    app = QApplication(sys.argv);
    w = MapWidget();
    w.show();
    sys.exit(app.exec_());
