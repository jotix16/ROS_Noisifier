import sys
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton, QButtonGroup

from layout import Ui_MainWindow
from tab_start import Ui_Form

import os
from threading import Thread
from subprocess import check_call
from PyQt5 import QtGui

import rospy
from std_msgs.msg import Float64MultiArray
from launcher import *

PATHS = [
    "/home/mzhobro/filter_ws/src/localizationfusionlibrary/test/test1.bag",
    "/home/mzhobro/filter_ws/src/localizationfusionlibrary/test/test2.bag",
    "/home/mzhobro/filter_ws/src/localizationfusionlibrary/test/test3.bag"
]

class MainWindow(QtWidgets.QMainWindow,QtWidgets.QWidget, Ui_MainWindow):
    def __init__(self, set_variance="set_variance", parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.interactions()
        self.variance = [0]*12
        rospy.init_node('window', anonymous=True)
        self.set_covariance_pub = rospy.Publisher(set_variance, Float64MultiArray, queue_size=10)
        print("NEW NOISIFIER: ", set_variance)


    def interactions(self):
        ''' Set up the connectivity between all the GUI elements that where generated in the layout.py file.'''
        # Button to launch the rosnodes
        # self.Browse.clicked.connect(self.on_click_browse)
        # self.Launch.clicked.connect(self.on_click_launch)

        # Lambda was used here to demonstrate how a second argument can be passed to the handler function
        self.variance_x.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val*0.02, self.x, [0]))
        self.variance_y.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val*0.02, self.y, [1]))
        self.variance_z.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val*0.02, self.z, [2]))
        self.variance_roll.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val*0.01, self.roll, [3]))
        self.variance_pitch.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val*0.01, self.pitch, [4]))
        self.variance_yaw.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val*0.01, self.yaw, [5]))
        self.variance_linear_vel.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val*0.01, self.lin_vel, [6,7,8]))
        self.variance_angular_vel.sliderMoved.connect(lambda val, dummy=None: self.slider_moved_handler(val*0.01, self.angular, [9,10,11]))

    def slider_moved_handler(self, val, label, index):
        '''This function runs whenever the value of the slider moves'''
        label.setText(str(val))
        for ix in index:
            self.variance[ix] = val
        msg = Float64MultiArray()
        msg.data = [i for i in self.variance]
        self.set_covariance_pub.publish(msg)

class TabStart(QtWidgets.QWidget,Ui_Form):
    def __init__(self, parent=None):
        super(TabStart, self).__init__(parent)
        self.setupUi(self)

        self.bag_path = None
        self.not_launched = True
        self.odom_topic = None
        self.interactions()
        self.bag_index = -1
        self.nr = 0
        self.imu_topic_list = []

    def ready(self):
        return self.bag_path is not None and self.odom_topic is not None and self.nr>0

    def interactions(self):
        ''' Set up the connectivity between all the GUI elements that where generated in the layout.py file.'''
        # Button to launch the rosnodes
        self.Browse.clicked.connect(self.on_click_browse)
        self.Launch.setEnabled(False)
        self.Launch.clicked.connect(self.on_click_launch)
        # radio buttons
        self.buttonGroup.buttonClicked[int].connect(self.on_radio_button_clicked)
        # self.comboBox.setEnabled(False)
        self.comboBox_topics.activated[str].connect(self.topic_choice)

        for i in range(10):
            self.comboBox_nr.addItem(str(i))
        self.comboBox_nr.activated[int].connect(self.nr_choice)

    def nr_choice(self, nr):
        self.nr = int(nr)

    def topic_choice(self, topic):
        self.odom_topic = topic
        if self.ready():
            self.Launch.setEnabled(True)

    def on_radio_button_clicked(self, id):
        switcher = {
                "bag1": 0, # bag1
                "bag2": 1, # bag2
                "bag3": 2, # bag3
                "None": -1, # no bag
            }
        bag_index = switcher.get(self.buttonGroup.checkedButton().text(), None)
        if bag_index != self.bag_index:
            self.bag_index = bag_index

            if bag_index != -1:
                self.lineEdit.setText(PATHS[bag_index])
                self.bag_path = PATHS[bag_index]
                self.add_drop_down_topics()
            else:
                self.Launch.setEnabled(False)
                self.comboBox_topics.clear()
                self.imu_topic_list = []
                self.bag_path = None
                self.lineEdit.setText("")
            print("ODOM TOPIC", self.odom_topic)

    def on_click_browse(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","Bag Files (*.bag)", options=options)
        if fileName:
            print(fileName)
            self.bag_path = fileName
            print("THE BAGGG", self.bag_path)
            self.radio_None.setChecked(True)
            self.lineEdit.setText(fileName)
            self.add_drop_down_topics()

    def add_drop_down_topics(self):
        self.odom_topic = None
        self.comboBox_topics.clear()
        self.imu_topic_list = []
        self.comboBox_topics.addItem("Select Topic")
        odom_topic_list, self.imu_topic_list = list_of_odometry_topics(self.bag_path)
        for topic in odom_topic_list:
            self.comboBox_topics.addItem(topic)

    def on_click_launch(self):
        if self.not_launched:
            self.not_launched = False
            Thread(target=self.launch_handle).start()

    def launch_handle(self):
        bag_name = self.bag_path.split("/")[-1].split(".")[0] + "_noised"
        print(bag_name)
        # cmd = "source /home/mzhobro/evaluation/devel/setup.bash; roslaunch {}; mv ~/.ros/bag1_noise.bag ~/evaluation/noised_bags/{}.bag".format(self.bag_path, bag_name)
        # check_call(cmd, shell=True, executable="/bin/bash")
        start_launch(self.bag_path)

    def get_params(self):
        return self.odom_topic, self.nr, self.imu_topic_list

class Window(QtWidgets.QWidget):
    def __init__(self):
        super(Window, self).__init__()
        self.tabs = QtWidgets.QTabWidget()
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.tabs)
        # button = QtWidgets.QToolButton()
        # button.setToolTip('Add New Tab')
        # button.setText("start nodes")
        # button.clicked.connect(self.addNewTab)
        # button.setIcon(self.style().standardIcon(
        #     QtWidgets.QStyle.SP_DialogApplyButton))

        self.Launch = QtWidgets.QPushButton()
        self.Launch.setFixedSize(40, 23) #setGeometry(QtCore.QRect(210, 130, 121, 31))
        self.Launch.setStyleSheet("background-color: red")
        self.Launch.setObjectName("Launch")
        self.Launch.setEnabled(False)
        self.addFirstTab()
        self.tabs.setCornerWidget(self.Launch, QtCore.Qt.TopRightCorner)

        # self.tabs.widget(0).lineEdit.textChanged.connect(self.on_text_changed)
        self.tabs.widget(0).comboBox_topics.activated[str].connect(self.on_changed)
        self.tabs.widget(0).comboBox_nr.activated[int].connect(self.on_changed)
        self.Launch.clicked.connect(self.on_click_launch)

    @QtCore.pyqtSlot()
    def on_changed(self):
        if self.tabs.widget(0).ready():
            self.Launch.setStyleSheet("background-color: green")
            self.Launch.setEnabled(True)

    def addFirstTab(self):
        text = 'START TAB'
        self.tabs.addTab(TabStart(self.tabs), text)

    def on_click_launch(self):
        if self.tabs.widget(0).ready():
            topic, nr, imu_topic_list = self.tabs.widget(0).get_params()
            for i in range(nr):
                text = 'Tab %d' % (self.tabs.count())
                set_variance_s ="set_variance_"+str(i)
                self.tabs.addTab(MainWindow(set_variance=set_variance_s, parent=self.tabs), text)
                start_noisy_node(topic, "odometry/noised"+str(i), set_variance_s)
            self.tabs.widget(0).launch_handle()

if __name__ == '__main__':

    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.setGeometry(200, 200, 900, 500)
    window.show()
    sys.exit(app.exec_())
