#!/usr/bin/env python3
import sys, os
import rospy
from std_msgs.msg import Bool, String
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout,
    QLineEdit, QLabel, QHBoxLayout
)
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QSize, QTimer

class LoggerGUI(QWidget):
    def __init__(self):
        super().__init__()
        rospy.init_node("tocabi_logger_gui", anonymous=True)

        self.pub_toggle = rospy.Publisher("/tc_rec/is_logging", Bool, queue_size=1)
        self.pub_filename = rospy.Publisher("/tc_rec/record_filename", String, queue_size=1)
        rospy.Subscriber("/tc_rec/logger_status", String, self.status_cb)

        # ——— Load and set the window icon ———
        icon_path = os.path.join(os.path.dirname(__file__), "icons", "tocabi_rec.png")
        if os.path.exists(icon_path):
            self.setWindowIcon(QIcon(icon_path))
        else: print("Warning: icon not found at", icon_path)

        # UI setup
        self.setWindowTitle("Tocabi Logger Control")
        self.setFixedSize(300, 200)
        layout = QVBoxLayout()

        # Filename entry
        file_layout = QHBoxLayout()
        self.file_edit = QLineEdit(self)
        self.file_edit.setPlaceholderText("Enter filename.txt")
        file_layout.addWidget(QLabel("File:"))
        file_layout.addWidget(self.file_edit)
        layout.addLayout(file_layout)

        # Status label
        self.status_label = QLabel("Waiting for filename...")
        layout.addWidget(self.status_label)

        # Buttons
        btn_layout = QHBoxLayout()
        self.record_button = QPushButton()
        self.record_button.setIcon(QIcon(os.path.join(os.path.dirname(__file__),"icons/rec.png")))
        self.record_button.setIconSize(QSize(48,48))
        self.record_button.clicked.connect(self.on_record)
        btn_layout.addWidget(self.record_button)

        self.stop_button = QPushButton()
        self.stop_button.setIcon(QIcon(os.path.join(os.path.dirname(__file__),"icons/stop.png")))
        self.stop_button.setIconSize(QSize(48,48))
        self.stop_button.clicked.connect(self.on_stop)
        btn_layout.addWidget(self.stop_button)

        layout.addLayout(btn_layout)

        self.setLayout(layout)

        # Timer to catch ROS shutdown
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_ros)
        self.timer.start(100)

    def on_record(self):
        fn = self.file_edit.text().strip()
        if not fn:
            self.status_label.setText("Please enter filename")
            return
        # publish filename then toggle
        self.pub_filename.publish(fn)
        self.pub_toggle.publish(True)

    def on_stop(self):
        self.pub_toggle.publish(False)

    def status_cb(self, msg):
        # update GUI status
        self.status_label.setText(msg.data)

    def check_ros(self):
        if rospy.is_shutdown():
            self.close()

if __name__=="__main__":
    app = QApplication(sys.argv)
    win = LoggerGUI()
    win.show()
    sys.exit(app.exec_())
