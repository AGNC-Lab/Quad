#!/usr/bin/python
# -*- coding: utf-8 -*-

# Author:  Mario A Molina
# Purpose: To read from key_publisher and update a char 'x' in the window


import sys
import rospy
from std_msgs.msg import Int32
from PyQt4 import QtGui, QtCore

#Widget that draws and updates the char 'x'
class Example(QtGui.QWidget):
    
    def __init__(self):
        super(Example, self).__init__()  #constructs the parent class variables 
	
	#The x and y of the char 'x'
	self.positionX = 75
	self.positionY = 75
       
        self.initUI() #calls bottom def
        
    def initUI(self):      

        self.text = 'x' #the char 'x'

	#Window setup
        self.setGeometry(300, 300, 280, 170)
        self.setWindowTitle('Draw X')
        self.show()

	#This method updates the window (paints the window)
    def paintEvent(self, event):

        qp = QtGui.QPainter()      #makes the painter object (the object Example is the window and qp is the graphics that will be put on the window)
        qp.begin(self)             #turns graphics on if not on already
        self.drawText(qp)          #calls bottom and passes the graphics.
        
    def drawText(self, qp):
      
        qp.setPen(QtGui.QColor(168, 34, 3))                            #color of the char
        qp.setFont(QtGui.QFont('Decorative', 10))                      #font of the char
        qp.drawText(self.positionX ,self.positionY, self.text)         #draws text at certain position on the graphics

	
    #Updates the position of the char 'x'   y axis is flipped in graphics so higher number means more down
    def updatePosition(self, number):
	if(number == 1):
	    self.positionX -= 2

	if(number == 2):
	    self.positionY -= 2

	if(number == 3):
	    self.positionY += 2

	if(number == 4):
	    self.positionX += 2

	#this method name update is given to any object with a paintEvent on it and basically
	#just calls paintEvent... it is standarized never to call the paintEvent method directly
	self.update()  
    
#set-up    
def main():
    
    app = QtGui.QApplication(sys.argv)
    ex = Example()

    #ROS callback function... its placed here because it depends on the local object ex...
    def callback(msg):
	print msg.data
	ex.updatePosition(msg.data)    #calles update position with information of the key press


    #ROS init of node and subscriber
    rospy.init_node('X_Graphics')
    sub = rospy.Subscriber('key', Int32, callback)

    sys.exit(app.exec_())		#closes appliation when the x is pressed (!!!no lines of code will run after this command!!!)


#calls main only if this is the primary script
if __name__ == '__main__':
    main()
