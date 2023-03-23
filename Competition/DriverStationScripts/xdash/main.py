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
        self.sdb = self.inst.getTable("SmartDashboard")
        self.blackmesa = self.sdb.getSubTable("BlackMesa")
        self.subs = []
        self.ipWriter = IPWriter(self.inst, self.blackmesa, self.ntConnected)
        self.ipWriter.start()
        def setupAutonomousCommandNameListener(topic, listener):
            sub = self.sdb.getStringTopic(topic).subscribe("")
            self.subs.append(sub)
            self.inst.addListener(sub, nt.EventFlags.kValueAll, listener)
        def setupAutonomousOracleListenerBoolean(topic, listener):
            sub = self.sdb.getSubTable("AutonomousOracle").getBooleanTopic(topic).subscribe(False)
            self.subs.append(sub)
            self.inst.addListener(sub, nt.EventFlags.kValueAll, listener)
        def setupAutonomousOracleListenerString(topic, listener):
            sub = self.sdb.getSubTable("AutonomousOracle").getStringTopic(topic).subscribe("")
            self.subs.append(sub)
            self.inst.addListener(sub, nt.EventFlags.kValueAll, listener)
        def setupAutonomousOracleListenerDouble(topic, listener):
            sub = self.sdb.getSubTable("AutonomousOracle").getDoubleTopic(topic).subscribe(-488.0)
            self.subs.append(sub)
            self.inst.addListener(sub, nt.EventFlags.kValueAll, listener)
        def setupStringListener(topic, listener):
            sub = self.blackmesa.getStringTopic(topic).subscribe("")
            self.subs.append(sub)
            self.inst.addListener(sub, nt.EventFlags.kValueAll, listener)
        def setupIntegerListener(topic, listener):
            sub = self.blackmesa.getIntegerTopic(topic).subscribe(0)
            self.subs.append(sub)
            self.inst.addListener(sub, nt.EventFlags.kValueAll, listener)
        def setupDoubleListener(topic, listener):
            sub = self.sdb.getSubTable("PoseSubsystem").getDoubleTopic(topic).subscribe(0.0)
            self.subs.append(sub)
            self.inst.addListener(sub, nt.EventFlags.kValueAll, listener)
        setupStringListener("state", lambda event: self.streamingStatus.setText(event.data.value.getString()))
        setupStringListener("streamingError", lambda event: self.streamingError.setText(event.data.value.getString()))
        setupStringListener("streamingUrl", lambda event: self.streamingServerUrl.setText(event.data.value.getString()))
        setupStringListener("DriverStationIp", lambda event: self.driverStationIP.setText(event.data.value.getString()))
        setupDoubleListener("Time", lambda event: self.timeDisplay.setText(str(event.data.value.getDouble())))
        setupIntegerListener("streamingIteration", lambda event: self.streamingRestartCount.setText(str(event.data.value.getInteger())))
        setupIntegerListener("streamingForceRestartCount", lambda event: self.streamingForceRestartCount.setText(str(event.data.value.getInteger())))
        setupIntegerListener("roundCount", lambda event: self.round.setText(str(event.data.value.getInteger())))

        # Autonomous Oracle Listeners
        setupAutonomousCommandNameListener("Current autonomous command name", lambda event: self.currentAutonomousCommandName.setText(event.data.value.getString()))
        setupAutonomousOracleListenerBoolean("EnableAcquireGamePiece", lambda event: self.enableAcquireGamePiece.setText(str(event.data.value.getBoolean())))
        setupAutonomousOracleListenerBoolean("EnableBalance", lambda event: self.enableBalance.setText(str(event.data.value.getBoolean())))
        setupAutonomousOracleListenerBoolean("EnableDrivePhaseOne", lambda event: self.enableDrivePhaseOne.setText(str(event.data.value.getBoolean())))
        setupAutonomousOracleListenerBoolean("EnableMoveToScore", lambda event: self.enableMoveToScore.setText(str(event.data.value.getBoolean())))
        setupAutonomousOracleListenerBoolean("EnableSecondScore", lambda event: self.enableSecondScore.setText(str(event.data.value.getBoolean())))
        setupAutonomousOracleListenerString("InitialGamePiece", lambda event: self.initialGamePiece.setText(event.data.value.getString()))
        setupAutonomousOracleListenerDouble("InitialScoringLocationIndex", lambda event: self.initialScoringLocationIndex.setText(str(event.data.value.getDouble())))
        setupAutonomousOracleListenerString("InitialScoringMode", lambda event: self.initialScoringMode.setText(event.data.value.getString()))
        setupAutonomousOracleListenerString("Lane", lambda event: self.lane.setText(event.data.value.getString()))
        setupAutonomousOracleListenerString("MantlePrepPosition", lambda event: self.mantlePrepPosition.setText(event.data.value.getString()))
        setupAutonomousOracleListenerString("SecondGamePiece", lambda event: self.secondGamePiece.setText(event.data.value.getString()))
        setupAutonomousOracleListenerDouble("SecondScoringLocationIndex", lambda event: self.secondScoringLocationIndex.setText(str(event.data.value.getDouble())))
        setupAutonomousOracleListenerString("SecondScoringMode", lambda event: self.secondScoringMode.setText(event.data.value.getString()))

    def createUI(self):
        streamingTitleLabel = QLabel("Camera Streaming Controls")
        streamingTitleLabel.setFont(QFont("Times",pointSize=18, weight=1000))
        ntConnectedLabel = QLabel("Is Connected")
        self.ntConnected = QLabel()
        roundLabel = QLabel("Round Count")
        self.round = QLabel()
        timeLabel = QLabel()
        timeLabel.setText("Time")
        self.timeDisplay = QLabel()
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
        # Autonomous Layout
        autonomousTitleLabel = QLabel ("Autonomous Status")
        autonomousTitleLabel.setFont(QFont("Times",pointSize=18, weight=1000))
        
        currentAutonomousCommandNameLabel = QLabel("Current Auto Command")
        currentAutonomousCommandNameLabel.setFont(QFont("Times",pointSize=14, weight=2000))
        self.currentAutonomousCommandName = QLabel()
        self.currentAutonomousCommandName.setFont(QFont("Times",pointSize=14, weight=2000))

        enableAcquireGamePieceLabel = QLabel()
        enableAcquireGamePieceLabel.setText("Game Piece Acquisition Enabled")
        self.enableAcquireGamePiece = QLabel()

        enableBalanceLabel = QLabel()
        enableBalanceLabel.setText("Balancing Enabled")
        self.enableBalance = QLabel()

        enableDrivePhaseOneLabel = QLabel()
        enableDrivePhaseOneLabel.setText("Drive Phase One Enabled")
        self.enableDrivePhaseOne = QLabel()

        enableMoveToScoreLabel = QLabel()
        enableMoveToScoreLabel.setText("Moving to Score Enabled")
        self.enableMoveToScore = QLabel()

        enableSecondScoreLabel = QLabel()
        enableSecondScoreLabel.setText("Scoring Second Game Piece Enabled")
        self.enableSecondScore = QLabel()

        initialGamePieceLabel = QLabel()
        initialGamePieceLabel.setText("Initial Game Piece Type")
        self.initialGamePiece = QLabel()

        initialScoringLocationIndexLabel = QLabel()
        initialScoringLocationIndexLabel.setText("Initial Scoring Location")
        self.initialScoringLocationIndex = QLabel()

        initialScoringModeLabel = QLabel()
        initialScoringModeLabel.setText("Initial Scoring Mode")
        self.initialScoringMode = QLabel()

        laneLabel = QLabel()
        laneLabel.setText("Maneuver Lane")
        self.lane = QLabel()

        mantlePrepPositionLabel = QLabel()
        mantlePrepPositionLabel.setText("Charge Plate Mantling Side")
        self.mantlePrepPosition = QLabel()

        secondGamePieceLabel = QLabel()
        secondGamePieceLabel.setText("Second Game Piece Type")
        self.secondGamePiece = QLabel()

        secondScoringLocationIndexLabel = QLabel()
        secondScoringLocationIndexLabel.setText("Second Scoring Location")
        self.secondScoringLocationIndex = QLabel()
        
        secondScoringModeLabel = QLabel()
        secondScoringModeLabel.setText("Second Scoring Mode")
        self.secondScoringMode = QLabel()

        self.layout = QGridLayout()
        self.layout.setSizeConstraint(QLayout.SetMaximumSize)
        row = 1
        self.layout.addWidget(streamingTitleLabel, row, 1, 1, 2)
        row += 1
        def addPair(a, b):
            nonlocal row
            self.layout.addWidget(a, row, 1)
            self.layout.addWidget(b, row, 2)
            row += 1
        addPair(ntConnectedLabel, self.ntConnected)
        addPair(roundLabel, self.round)
        addPair(timeLabel, self.timeDisplay)
        addPair(driverStationIPLabel, self.driverStationIP)
        addPair(streamingStatusLabel, self.streamingStatus)
        addPair(streamingErrorLabel, self.streamingError)
        addPair(streamingServerUrlLabel, self.streamingServerUrl)
        addPair(streamingRestartCountLabel, self.streamingRestartCount)
        addPair(streamingForceRestartCountLabel, self.streamingForceRestartCount)
        self.layout.addWidget(self.forceRestartStreamingButton, row, 1, 1, 2)
        row += 2
        self.layout.addWidget(autonomousTitleLabel, row, 1, 1, 2)
        row += 1
        addPair(currentAutonomousCommandNameLabel, self.currentAutonomousCommandName)
        addPair(enableAcquireGamePieceLabel, self.enableAcquireGamePiece)
        addPair(enableBalanceLabel, self.enableBalance)
        addPair(enableDrivePhaseOneLabel, self.enableDrivePhaseOne)
        addPair(enableMoveToScoreLabel, self.enableMoveToScore)
        addPair(enableSecondScoreLabel, self.enableSecondScore)
        addPair(initialGamePieceLabel, self.initialGamePiece)
        addPair(initialScoringLocationIndexLabel, self.initialScoringLocationIndex)
        addPair(initialScoringModeLabel, self.initialScoringMode)
        addPair(laneLabel, self.lane)
        addPair(mantlePrepPositionLabel, self.mantlePrepPosition)
        addPair(secondGamePieceLabel, self.secondGamePiece)
        addPair(secondScoringLocationIndexLabel, self.secondScoringLocationIndex)
        addPair(secondScoringModeLabel, self.secondScoringMode)

        self.centralWidget = QWidget()
        self.centralWidget.setLayout(self.layout)
        self.setCentralWidget(self.centralWidget)
    
    def forceRestartStreaming(self):
        restartCounter = self.blackmesa.getEntry("streamingForceRestartCount").getInteger(0)
        self.blackmesa.getEntry("streamingForceRestartCount").setInteger(restartCounter + 1)

    def closeEvent(self, event):
        print("CLOSE")
        self.ipWriter.interrupt()
        event.accept()

class IPWriter(QThread):
    def __init__(self, inst, blackmesa, ntConnectedStatus):
        QThread.__init__(self)
        self.active = True
        self.blackmesa = blackmesa
        self.inst = inst
        self.ntConnectedStatus = ntConnectedStatus

    def run(self):
        while self.active:
            try:
                hostname = socket.gethostname()
                hostip = socket.gethostbyname(hostname)
            except:
                print("Unable to get Hostname and IP")
                hostname = None
                hostip = None
            self.blackmesa.putString("DriverStationIp", hostip)
            self.ntConnectedStatus.setText(str(self.inst.isConnected()))
            time.sleep(1)
    
    def interrupt(self):
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