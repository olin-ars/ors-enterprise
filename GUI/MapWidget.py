import sys

from PyQt4.QtGui import *
from PyQt4.QtCore import *

class MapWidget(QFrame):
    """Map Widget : Coordinates from -1.0 to 1.0"""
    def __init__(self,parent=None):
        QFrame.__init__(self,parent); #creating map frame
        self.location = QPointF(0.5,0.5);  #sets locatio to a certain point
        self.locPen = QPen(QColor.fromRgb(255,255,255)); #sets up pen
        self.locPen.setWidthF(0.1); #sets up size of pen
    def setLoc(self,*args): 
        """setLoc : set latitude/longitude
        setLoc(int latitude, int longitude);
        setLoc(QPoint)
        setLoc(QPointF)
        """
        # from -1 to 1
        self.location = QPointF(args[1],-args[0]); #set location to inputted values <- this is werid, why is it inverting the second input
    def drawBK(self,p): #drawing the background map
        p.translate(1,1);
        m = QPixmap('map.jpg'); #assigning image for background
        p.drawPixmap(QRectF(-1,-1,2,2),m,QRectF(0,0,m.width(),m.height())); #setting the size of the map background
    def drawLocation(self,p): #putting a point at the location of the boat (shown as a box)
        p.setPen(self.locPen); #settign the pen for this partucular instance
        p.drawPoint(self.location); #placing point at location
    def paintEvent(self,event): #makes the full image
        p = QPainter(self); #allows us to paint
        p.scale(self.width()/2,self.height()/2); #viewport from -1~1 ->sets the size of the window
        self.drawBK(p); #calling previous function to draw background map
        self.drawLocation(p); #calling previous function to put point on image
        QFrame.paintEvent(self,event); #
    def mousePressEvent(self,event):
        pass

if __name__ == "__main__":
    global app;
    app = QApplication(sys.argv);
    w = MapWidget();
    w.show();
    sys.exit(app.exec_());
