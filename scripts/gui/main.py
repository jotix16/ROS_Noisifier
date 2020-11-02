#!/usr/bin/env python
import sys
import os
from threading import Thread
from subprocess import check_call
import PyQt5
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
from layout import Ui_MainWindow

import rospy
from std_msgs.msg import Float64MultiArray

PATHS = [
    "/home/mzhobro/evaluation/src/noisy/launch/bag1.launch",
    "/home/mzhobro/evaluation/src/noisy/launch/bag2.launch",
    "/home/mzhobro/evaluation/src/noisy/launch/bag3.launch"
]

class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        self.not_launched = True
        super(MainWindow, self).__init__()
        self.setupUi(self)
        self.interactions()
        self.variance = [0]*12
        rospy.init_node('window', anonymous=True)
        self.set_covariance_pub = rospy.Publisher('set_variance', Float64MultiArray, queue_size=10)
        self.bag_index = None
        self.bag_path = None

    def interactions(self):
        '''
            Set up the connectivity between all the GUI elements that where generated in
            the layout.py file.
        '''
        # Button to launch the rosnodes
        self.Browse.clicked.connect(self.on_click_browse)
        self.Launch.clicked.connect(self.on_click_launch)

        # Lambda was used here to demonstrate how a second argument can be passed to the handler function
        self.variance_x.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val, self.x, [0]))
        self.variance_y.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val, self.y, [1]))
        self.variance_z.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val, self.z, [2]))
        self.variance_roll.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val, self.roll, [3]))
        self.variance_pitch.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val, self.pitch, [4]))
        self.variance_yaw.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val, self.yaw, [5]))
        self.variance_linear_vel.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val, self.lin_vel, [6,7,8]))
        self.variance_angular_vel.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val, self.angular, [9,10,11]))
        # self.variance_x.sliderReleased.connect(self.slider_released_handler)

        # radio buttons
        self.buttonGroup.buttonClicked[int].connect(self.on_radio_button_clicked)

    def on_radio_button_clicked(self, id):
        switcher = {
                "bag1": 0, # bag1
                "bag2": 1, # bag2
                "bag3": 2, # bag3
            }

        self.bag_index = switcher.get(self.buttonGroup.checkedButton().text(), None)
        print("BAAAAAAAAAAAG", " -> ", self.bag_index, self.buttonGroup.checkedButton().text())

    def slider_moved_handler(self, val, label, index):
        '''
            Takes the slider value and a dummy value and sets the label of the variance in the interface
            (This function runs whenever the value of the slider moves)
        '''
        val_new =  val * 0.02
        for ix in index:
            self.variance[ix] = val_new
        msg = Float64MultiArray()
        msg.data = [i for i in self.variance]
        self.set_covariance_pub.publish(msg)

    def on_click_browse(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","Launch Files (*.launch)", options=options)
        if fileName:
            print(fileName)
            self.bag_path = fileName
            self.lineEdit.setText(fileName)

    def on_click_launch(self):
        # if radio button checked than choose from there
        # otherwise from the edit field
        # if both empty do nothing
        if self.not_launched and self.bag_index is not None:
            self.not_launched = False
            self.bag_path = PATHS[self.bag_index]
            Thread(target=self.launch_handle).start()
        elif self.not_launched and self.bag_path is not None:
            self.not_launched = False
            Thread(target=self.launch_handle).start()

    def launch_handle(self):
        bag_name = self.bag_path.split("/")[-1].split(".")[0] + "_noised"
        print(bag_name)
        cmd = "source /home/mzhobro/evaluation/devel/setup.bash; roslaunch {}; mv ~/.ros/bag1_noise.bag ~/evaluation/noised_bags/{}.bag".format(self.bag_path, bag_name)
        check_call( cmd, shell=True, executable="/bin/bash")

def Window():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    Window()

#TO_DO:
# 1. Allow TO give topics through gui
# 3. give a name for the saving file through gui
#