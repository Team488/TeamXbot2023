from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import ntcore as nt
import sys
import os
import time
import socket

class Window(QMainWindow):

    def __init__(self, networkTableInstance, parent=None):
        super().__init__(parent)
        self.inst = networkTableInstance
        self.setWindowTitle("XDash")
        self.createUI()
        self.createNetworkTables()
    
    def createNetworkTables(self):
        self.blackmesa = self.inst.getTable("SmartDashboard").getSubTable("BlackMesa")
        self.subs = []
        self.ipWriter = IPWriter(self.blackmesa)
        self.ipWriter.start()
        def setupStringListener(topic, listener):
            sub = self.blackmesa.getStringTopic(topic).subscribe("")
            self.subs.append(sub)
            self.inst.addListener(sub, nt.EventFlags.kValueAll, listener)
        def setupIntegerListener(topic, listener):
            sub = self.blackmesa.getIntegerTopic(topic).subscribe(0)
            self.subs.append(sub)
            self.inst.addListener(sub, nt.EventFlags.kValueAll, listener)
        setupStringListener("state", lambda event: self.streamingStatus.setText(event.data.value.getString()))
        setupStringListener("streamingError", lambda event: self.streamingError.setText(event.data.value.getString()))
        setupStringListener("streamingUrl", lambda event: self.streamingStatus.setText(event.data.value.getString()))
        setupStringListener("streamingIteration", lambda event: self.streamingRestartCount.setText(event.data.value.getString()))
        setupIntegerListener("streamingForceRestartCount", lambda event: self.streamingForceRestartCount.setText(event.data.value.getInteger()))
    
    def createUI(self):
        driverStationIPLabel = QLabel()
        driverStationIPLabel.setText("Driver Station IP")
        self.driverStationIP = QLabel()
        streamingStatusLabel = QLabel()
        streamingStatusLabel.setText("Streaming Status")
        self.streamingStatus = QLabel()
        streamingErrorLabel = QLabel()
        streamingErrorLabel.setText("Streaming Error")
        self.streamingError = QLabel()
        streamingServerUrlLabel = QLabel()
        streamingServerUrlLabel.setText("Streaming Server URL")
        self.streamingServerUrl = QLabel()
        streamingRestartCountLabel = QLabel()
        streamingRestartCountLabel.setText("Streaming Restart Count")
        self.streamingRestartCount = QLabel()
        streamingForceRestartCountLabel = QLabel()
        streamingForceRestartCountLabel.setText("Streaming Force Restart Count")
        self.streamingForceRestartCount = QLabel()
        self.forceRestartStreamingButton = QPushButton()
        self.forceRestartStreamingButton.setText("Force Restart Streaming")
        self.forceRestartStreamingButton.clicked.connect(self.forceRestartStreaming)
        self.layout = QGridLayout()
        self.layout.setSizeConstraint(QLayout.SetMaximumSize)
        self.layout.addWidget(driverStationIPLabel, 1, 1)
        self.layout.addWidget(self.driverStationIP, 1, 2)
        self.layout.addWidget(streamingStatusLabel, 2, 1)
        self.layout.addWidget(self.streamingStatus, 2, 2)
        self.layout.addWidget(streamingErrorLabel, 3, 1)
        self.layout.addWidget(self.streamingError, 3, 2)
        self.layout.addWidget(streamingServerUrlLabel, 4, 1)
        self.layout.addWidget(self.streamingServerUrl, 4, 2)
        self.layout.addWidget(streamingRestartCountLabel, 5, 1)
        self.layout.addWidget(self.streamingRestartCount, 5, 2)
        self.layout.addWidget(streamingForceRestartCountLabel, 6, 1)
        self.layout.addWidget(self.streamingForceRestartCount, 6, 2)
        self.layout.addWidget(self.forceRestartStreamingButton, 7, 1, 1, 2)
        self.centralWidget = QWidget()
        self.centralWidget.setLayout(self.layout)
        self.setCentralWidget(self.centralWidget)
    
    def forceRestartStreaming(self):
        restartCounter = self.blackmesa.getEntry("streamingForceRestartCount").getInteger(0)
        self.blackmesa.getEntry("streamingForceRestartCount").setInteger(restartCounter + 1)

    def closeEvent(self, event):

        event.accept()

class IPWriter(QThread):
    def __init__(self, blackmesa):
        QThread.__init__(self)
        self.active = True
        self.blackmesa = blackmesa

    def run(self):
        while self.active:
            try:
                hostname = socket.gethostname()
                hostip = socket.gethostbyname(hostname)
            except:
                print("Unable to get Hostname and IP")
                hostname = None
                hostip = None
            time.sleep(1)

        blackmesa.putString("DriverStationIp", hostip)
    
    def interrupt():
        self.active = False

def main():
    inst = nt.NetworkTableInstance.create()
    inst.startClient4("xdash")
    inst.setServerTeam(488) # replace this with inst.setServer(127.0.0.1) if it's running locally
    app = QApplication(sys.argv)
    main = Window(networkTableInstance=inst)
    main.show()
    result = app.exec_()
    #Exiting and running ntcore cleanup handlers is slow, you probably need ctrl+c to force quit with sys.exit
    #sys.exit(result)
    os._exit(result)

if __name__ == '__main__':
    main()