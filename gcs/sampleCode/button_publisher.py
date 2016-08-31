#!/usr/bin/env python

#ROS imports
import rospy
from std_msgs.msg import Int32

#PyQt Imports
import sys
from PyQt4 import QtGui, QtCore

#ROS CODE
rospy.init_node('slider')
pub = rospy.Publisher('number', Int32, queue_size=10)
rate = rospy.Rate(2)


#Button Code
class Example(QtGui.QWidget):
    
    def __init__(self):
        super(Example, self).__init__()   #Calls parent constructor
        
        self.initUI()                     #Calls bottom

    def handleButton(self, value):
	print "Sent Value: %s" % value
	pub.publish(value)                #Publishes value when button is clicked
        
    def initUI(self):
        
        #initialized a number display a slider and a button 
        lcd = QtGui.QLCDNumber(self)
        sld = QtGui.QSlider(QtCore.Qt.Horizontal, self)
	button = QtGui.QPushButton('Send', self)

	#addes all of the widgets to a VBox
        vbox = QtGui.QVBoxLayout()
        vbox.addWidget(lcd)
        vbox.addWidget(sld)
	vbox.addWidget(button)

        self.setLayout(vbox)    #puts the vbox on the window
        sld.valueChanged.connect(lcd.display)   #updates value on display when slider is moved
	
	button.clicked.connect(lambda: self.handleButton(lcd.intValue()))     #sends value on display when button is pressed
        
	#sets up the screen size and name
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Signal Sender')
        self.show()
        
def main():
    
    #Sets up window
    app = QtGui.QApplication(sys.argv)
    ex = Example()

    sys.exit(app.exec_())                           #closes appliation when the x is pressed (!!!no lines of code will run after this command!!!)

#calls main only if this is the primary script
if __name__ == '__main__':
    main()

