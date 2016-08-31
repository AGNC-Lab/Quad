#!/usr/bin/env python

#ROS imports
import rospy
from std_msgs.msg import Int32

#PyQt Imports
import sys
from PyQt4 import QtGui, QtCore


#Screen_Code
class Example(QtGui.QWidget): 
      
    def __init__(self):
        super(Example, self).__init__()    #Calls parent constructor
        self.lcd = QtGui.QLCDNumber()   #makes a display that is  viewable to all the class functions
        self.initUI()   #Calls bottom

	        
    def initUI(self):

	#addes all of the widgets to a VBox
        vbox = QtGui.QVBoxLayout()
        vbox.addWidget(self.lcd)
	
	#adds box to the window and sets up the screen size and name 
        self.setLayout(vbox)        
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Recieved Signal')
        self.show()

    #function called when a value is recieved
    def updateLCD(self, value):
	self.lcd.display(value) #updates the display

        
def main():
    
    #Sets up window
    app = QtGui.QApplication(sys.argv)
    ex = Example()
    
    #ROS CODE
    def callback(value):
	ex.updateLCD(value.data)

    rospy.init_node('button_subscriber')
    sub = rospy.Subscriber('number', Int32, callback)

    sys.exit(app.exec_())                   #closes appliation when the x is pressed (!!!no lines of code will run after this command!!!)
    

#calls main only if this is the primary script
if __name__ == '__main__':
    main()

