# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './pnp.ui'
#
# Created: Sun Jul 28 23:47:34 2013
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1120, 971)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.failButton = QtGui.QPushButton(self.centralwidget)
        self.failButton.setGeometry(QtCore.QRect(320, 850, 151, 71))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.failButton.setFont(font)
        self.failButton.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.failButton.setAutoFillBackground(False)
        self.failButton.setStyleSheet(_fromUtf8("* { background-color: rgb(211, 38, 38) }\n"
"* { color: rgb(255,255, 255) }\n"
""))
        self.failButton.setObjectName(_fromUtf8("failButton"))
        self.passButton = QtGui.QPushButton(self.centralwidget)
        self.passButton.setGeometry(QtCore.QRect(650, 850, 151, 71))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.passButton.setFont(font)
        self.passButton.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.passButton.setStyleSheet(_fromUtf8("* {background-color : rgb(82, 145, 73)}\n"
"* {color : rgb(255,255,255)}\n"
"\n"
""))
        self.passButton.setObjectName(_fromUtf8("passButton"))
        self.pnpButton = QtGui.QPushButton(self.centralwidget)
        self.pnpButton.setGeometry(QtCore.QRect(490, 850, 141, 71))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.pnpButton.setFont(font)
        self.pnpButton.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.pnpButton.setStyleSheet(_fromUtf8("* { background-color: rgb(234, 179, 48) }\n"
"* { color: rgb(255,255, 255) }\n"
""))
        self.pnpButton.setObjectName(_fromUtf8("pnpButton"))
        self.imgLabel = QtGui.QLabel(self.centralwidget)
        self.imgLabel.setGeometry(QtCore.QRect(50, 40, 1024, 768))
        self.imgLabel.setFrameShape(QtGui.QFrame.Box)
        self.imgLabel.setText(_fromUtf8(""))
        self.imgLabel.setObjectName(_fromUtf8("imgLabel"))
        self.runLabel = QtGui.QLabel(self.centralwidget)
        self.runLabel.setGeometry(QtCore.QRect(100, 860, 151, 61))
        self.runLabel.setText(_fromUtf8(""))
        self.runLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.runLabel.setObjectName(_fromUtf8("runLabel"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "P/NP", None, QtGui.QApplication.UnicodeUTF8))
        self.failButton.setText(QtGui.QApplication.translate("MainWindow", "Fail", None, QtGui.QApplication.UnicodeUTF8))
        self.passButton.setText(QtGui.QApplication.translate("MainWindow", "Pass", None, QtGui.QApplication.UnicodeUTF8))
        self.pnpButton.setText(QtGui.QApplication.translate("MainWindow", "P/NP", None, QtGui.QApplication.UnicodeUTF8))

