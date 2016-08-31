#!/usr/bin/env python

# Author:  Mario A Molina
# Purpose: To publish information through key presses that will update
#          the position of the char 'x' in image_updater
# !!!READ  click on the window that pops up and press your keys there   !!!READ

import rospy
import sys

from PyQt4 import QtCore, QtGui
from std_msgs.msg import Int32

#The event-handler widget/class
class Example(QtGui.QWidget):
    
    def __init__(self):
        super(Example, self).__init__()  #constructs the parent class variables      
	self.initUI()    		 #calls bottom 

    #This function just sets the window where you will input the key presses
    def initUI(self):

        self.setGeometry(300,300,250,150)        #Size of window
        self.setWindowTitle('Event handler')     #Name of window
        self.show()                              #shows the window on the screen


    #This function is called when an key is pressed while in the Event handler window
    #e is the event and holds the key pressed, the rest is self explanatory
    def keyPressEvent(self, e): 
        if e.key() == QtCore.Qt.Key_Up:
	        pub.publish(2)
	if e.key() == QtCore.Qt.Key_Down:
	        pub.publish(3)
	if e.key() == QtCore.Qt.Key_Left:
	        pub.publish(1)
	if e.key() == QtCore.Qt.Key_Right:
	        pub.publish(4)


#ROS setup code for node and publisher	
rospy.init_node('topic_publisher')
pub = rospy.Publisher('key', Int32, queue_size=10)
rate = rospy.Rate(30)                              #set rate higher so that the 'x' would update faster 

#application setup for class 
app = QtGui.QApplication(sys.argv)
ex = Example()
print "Click on the Window to input keys"
print "Use the arrow keys!!!"

sys.exit(app.exec_())                              #closes appliation when the x is pressed (!!!no lines of code will run after this command!!!)

