from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import sys

class Window(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("XDash")
        self.createUI()
    
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
        self.forceRestartStreaming = QPushButton()
        self.forceRestartStreaming.setText("Force Restart Streaming")
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
        self.layout.addWidget(self.forceRestartStreaming, 7, 1, 1, 2)
        self.centralWidget = QWidget()
        self.centralWidget.setLayout(self.layout)
        self.setCentralWidget(self.centralWidget)

def main():
    app = QApplication(sys.argv)
    main = Window()
    main.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()