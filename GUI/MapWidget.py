import sys

from PyQt4.QtGui import *
from PyQt4.QtCore import *

class MapWidget(QFrame):
    """Map Widget : Coordinates from -1.0 to 1.0"""
    def __init__(self,parent=None):
        QFrame.__init__(self,parent);
        self.location = QPointF(0.5,0.5);
        self.locPen = QPen(QColor.fromRgb(255,255,255));
        self.locPen.setWidthF(0.1);
    def setLoc(self,*args):
        """setLoc : set latitude/longitude
        setLoc(int latitude, int longitude);
        setLoc(QPoint)
        setLoc(QPointF)
        """
        # from -1 to 1
        self.location = QPointF(args[1],-args[0]);
    def drawBK(self,p):
        p.translate(1,1);
        m = QPixmap('map.jpg');
        p.drawPixmap(QRectF(-1,-1,2,2),m,QRectF(0,0,m.width(),m.height()));
    def drawLocation(self,p):
        p.setPen(self.locPen);
        p.drawPoint(self.location);
    def paintEvent(self,event):
        p = QPainter(self);
        p.scale(self.width()/2,self.height()/2); #viewport from -1~1
        self.drawBK(p);
        self.drawLocation(p);
        QFrame.paintEvent(self,event);
    def mousePressEvent(self,event):
        pass

if __name__ == "__main__":
    global app;
    app = QApplication(sys.argv);
    w = MapWidget();
    w.show();
    sys.exit(app.exec_());
