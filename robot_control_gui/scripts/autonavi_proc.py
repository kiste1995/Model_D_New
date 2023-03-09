#!/usr/bin/env python

import os, sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt

from std_msgs.msg import Bool, String, UInt8
from std_msgs.msg import UInt16, UInt64, Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16


class Thread_AutoNavigation(QThread):

    def __init__(self, parent = None):
        super(Thread_AutoNavigation, self).__init__(parent)

