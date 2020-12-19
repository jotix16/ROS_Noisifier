import sys
from PyQt5 import QtCore, QtWidgets, QtCore
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton, QButtonGroup

from layout import Ui_MainWindow
from tab_start import Ui_Form

import os
from threading import Thread
from subprocess import check_call
from PyQt5 import QtGui
import signal
from os.path import expanduser
import rospy
from std_msgs.msg import Float64MultiArray
from launcher import record_bag, play_bag, start_noisy_node, list_of_odometry_topics, start_roscore

PATHS = [
    os.path.dirname(os.path.realpath(__file__))+ "/../" + "test1.bag",
    os.path.dirname(os.path.realpath(__file__))+ "/../" + "test2.bag",
    os.path.dirname(os.path.realpath(__file__))+ "/../" + "test3.bag"
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

        self.out_bag_path = expanduser("~") +"/.ros"
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
        self.Browse_out.clicked.connect(self.on_click_browse_out)
        self.Launch.setEnabled(False)
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

    def on_click_browse(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","Bag Files (*.bag)", options=options)
        if fileName:
            self.bag_path = fileName
            self.radio_None.setChecked(True)
            self.lineEdit.setText(fileName)
            self.add_drop_down_topics()

    def on_click_browse_out(self):
        output_dir = QtWidgets.QFileDialog.getExistingDirectory(None, 'Select a folder:', expanduser("~"))
        if output_dir:
            self.out_bag_path = output_dir
            self.lineEdit_out.setText(output_dir)

    def add_drop_down_topics(self):
        self.odom_topic = None
        self.comboBox_topics.clear()
        self.imu_topic_list = []
        self.comboBox_topics.addItem("Select Topic")
        odom_topic_list, self.imu_topic_list = list_of_odometry_topics(self.bag_path)
        for topic in odom_topic_list:
            self.comboBox_topics.addItem(topic)

    def get_params(self):
        return self.odom_topic, self.nr, self.imu_topic_list, self.bag_path, self.out_bag_path

class Window(QtWidgets.QWidget):
    def __init__(self):
        super(Window, self).__init__()
        self.tabs = QtWidgets.QTabWidget()
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.tabs)

        self.addFirstTab()

        self.Launch = self.tabs.widget(0).Launch
        self.tabs.widget(0).comboBox_topics.activated[str].connect(self.on_changed)
        self.tabs.widget(0).comboBox_nr.activated[int].connect(self.on_changed)
        self.Launch.clicked.connect(self.on_click_launch)

        self.processes_to_be_killed_after_rosbag_play = []
        self.rosbag_play_process = None

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
            topic, nr, imu_topic_list, bag_path, out_bag_path = self.tabs.widget(0).get_params()
            noised_odom_topics = []

            self.processes_to_be_killed_after_rosbag_play.append(start_roscore())
            for i in range(nr):
                text = 'Noised Odom %d' % (self.tabs.count())
                set_variance_s ="set_variance_"+str(i)
                noised_odom_topic = "/odometry/noised"+str(i)
                noised_odom_topics.append(noised_odom_topic)

                self.tabs.addTab(MainWindow(set_variance=set_variance_s, parent=self.tabs), text)
                self.processes_to_be_killed_after_rosbag_play.append(start_noisy_node(topic, noised_odom_topic, set_variance_s))
            out_bag_name = bag_path.split("/")[-1].split(".")[0] + "_noised_x_"+str(nr)
            self.processes_to_be_killed_after_rosbag_play.append(record_bag([str(topic)] + imu_topic_list + noised_odom_topics, out_bag_path + "/"  + out_bag_name, out_bag_name))
            self.rosbag_play_process = play_bag(bag_path)
            self.wait_till_finish_thread()

    def wait_till_finish_thread(self):
        def run_in_thread():
            self.rosbag_play_process.wait()
            self.rosbag_play_process = None
            self.kill_subprocesses()
            self.close()
        Thread(target=run_in_thread).start()

    def kill_subprocesses(self):

        for ix, ps in reversed(list(enumerate(self.processes_to_be_killed_after_rosbag_play))):
            if ps is not None:
                print("KIEEEEEEEEEEEEEEEEEEELLING PROCESSS", ps.pid)
                self.processes_to_be_killed_after_rosbag_play[ix] = None
                # ps.kill()
                os.killpg(os.getpgid(ps.pid), signal.SIGINT)
                ps.wait()

        if self.rosbag_play_process is not None:
            print("---------------------------->Killing rosbag play", self.rosbag_play_process.pid)
            os.killpg(os.getpgid(self.rosbag_play_process.pid), signal.SIGINT)
            self.rosbag_play_process.wait()

    def closeEvent(self, event):
        # reply = QMessageBox.question(self, 'Window Close', 'Are you sure you want to close the window?',
        #         QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        self.kill_subprocesses()
        event.accept()
        print('Window closed')
        # if reply == QMessageBox.Yes:
        #     event.accept()
        # else:
        #     event.ignore()

    def sigint_handler(self, *arg):
        print("KILLING CTRL+C")
        self.close()

if __name__ == '__main__':

    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    signal.signal(signal.SIGINT, window.sigint_handler)
    window.setGeometry(200, 200, 900, 500)
    window.show()
    timer = QTimer()
    timer.start(500)  # You may change this if you wish.
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.
    sys.exit(app.exec_())
