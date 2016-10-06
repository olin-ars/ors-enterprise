import sys

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from math import *

def getPoint(r, theta): #in angle
#returns an x and y coordinate based on polar coordinates
    theta = radians(theta);
    return QPointF(r*sin(theta),-r*cos(theta));

class CompassWidget(QWidget):
    """ CompassWidget : angle given in degrees"""
    def __init__(self,parent=None):
        QWidget.__init__(self,parent);
        self.value = 0;
        self.maxVal = 360;
        self.minVal = 0;
        self.bigNotch = QPainterPath();
        self.smallNotch = QPainterPath();
    
        for i in range(12):
            r = 0.8 if i%3 ==0 else 0.5;

            p0 = QPointF(0,0);
            pL = getPoint(r/8,i*30-60);
            pM = getPoint(r,i*30);
            pR = getPoint(r/8,i*30+60);
            if i%3 == 0:
                self.bigNotch.addPolygon(QPolygonF([p0,pL,pM,pR,p0]));
            else:
                self.smallNotch.addPolygon(QPolygonF([p0,pL,pM,pR,p0]));
    
    def setValue(self,value):
        #values above the max value loop to the min value
        #values below the min value loop to the max value
        value = int(value);
        if value < self.minVal or value > self.maxVal:
            value = value % (self.maxVal - self.minVal);
        self.value = value;
    def drawBk(self,p):
        #draws the background of a compass?
        bkBrush = QBrush(QColor.fromRgbF(0,0,0,0.8), style = Qt.SolidPattern);
        bkPen = QPen(QColor.fromRgbF(0.7,0.7,0.7,1.0));
        bkPen.setWidthF(0.03);
        p.setBrush(bkBrush);
        p.setPen(bkPen)
        p.drawEllipse(-1,-1,2,2);
        bkBrush.setColor(QColor.fromRgbF(0.6,0.6,0.6,0.7));
        p.setBrush(bkBrush);
        p.drawEllipse(QRectF(-0.8,-0.8,1.6,1.6));
        
        bBrush = QBrush(QColor.fromRgbF(0.1,0.1,0.1,0.7));
        bPen = QPen(QColor.fromRgbF(0.9,0.9,0.9,1.0));
        bPen.setWidthF(0.01);
        p.setPen(bPen);
        p.setBrush(bBrush);
        p.drawPath(self.bigNotch);

        sBrush = QBrush(QColor.fromRgbF(0.6,0.6,0.6,0.5));
        sPen = QPen(QColor.fromRgbF(0.8,0.8,0.8,0.5));
        sPen.setWidthF(0.02);
        p.setPen(sPen);
        p.setBrush(sBrush);
        p.drawPath(self.smallNotch);
    def drawNeedle(self,p):
        p.setBrush(QBrush(QColor.fromRgbF(0.7,0.3,0.2,0.9)));
        pen = p.pen();
        p.setPen(pen);
        pen.setStyle(Qt.SolidLine);
        p.drawEllipse(QRectF(-0.1,-0.1,0.2,0.2));
        dstL = getPoint(0.2,self.value - 70);
        dstM = getPoint(0.6,self.value);
        dstR = getPoint(0.2,self.value + 70);
        pen.setStyle(Qt.NoPen);
        p.setPen(pen);
        p.drawConvexPolygon(QPointF(0,0),dstL,dstM,dstR);
    def paintEvent(self,event):
        p = QPainter(self);
        w = self.width();
        h = self.height();
        r = (min(w,h)-1)/2;
        p.translate(w/2,h/2);
        p.drawPoint(0,0);
        
        p.scale(r,r);
        self.drawBk(p);
        self.drawNeedle(p);

if __name__ == "__main__":
    global app;
    app = QApplication(sys.argv);
    w = MapWidget();
    w.show();
    sys.exit(app.exec_());
