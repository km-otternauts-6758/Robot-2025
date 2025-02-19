from math import pi
import time
import sys

import wpilib
from networktables import NetworkTables
from phoenix6 import SignalLogger
from rev import SparkMax
from wpilib import AddressableLED, Color, DigitalInput, LEDPattern, LiveWindow
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrapezoidProfile
from wpiutil import SendableRegistry

from AutoSequencerV2.autoSequencer import AutoSequencer
from dashboard import Dashboard
from drivetrain.controlStrategies.autoDrive import AutoDrive
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainControl import DrivetrainControl
from humanInterface.driverInterface import DriverInterface

# from humanInterface.ledControl import LEDControl
from navigation.forceGenerators import PointObstacle
from subsystems import Components, limelight
from utils.calibration import CalibrationWrangler
from utils.crashLogger import CrashLogger
from utils.faults import FaultWrangler
from utils.powerMonitor import PowerMonitor
from utils.rioMonitor import RIOMonitor
from utils.segmentTimeTracker import SegmentTimeTracker
from utils.signalLogging import logUpdate
from utils.singleton import destroyAllSingletonInstances
from webserver.webserver import Webserver

kLEDBuffer = 150


class MyRobot(wpilib.TimedRobot):
    #########################################################
    ## Common init/update for all modes
    def robotInit(self):
        # Since we're defining a bunch of new things here, tell pylint
        # to ignore these instantiations in a method.
        # pylint: disable=attribute-defined-outside-init
        self.enableLiveWindowInTest(True)
        self.state = True
        self.wristSetpoint = 0.52

        # LEDS
        self.led = wpilib.AddressableLED(0)
        self.ledData = [wpilib.AddressableLED.LEDData() for _ in range(kLEDBuffer)]
        self.rainbowFirstPixelHue = 0
        self.led.setLength(kLEDBuffer)
        self.led.setData(self.ledData)
        self.led.start()

        self.limitSwitch = DigitalInput(7)

        # Limelight
        NetworkTables.initialize(server="10.67.58.2")
        self.limeLight = NetworkTables.getTable("limelight-kmrobot")
        self.tx = self.limeLight.getEntry("tx")

        # self.tx = self.limeLight.putNumber("<tx>", 0)

        # Elevator
        self.elevator = Components.Elevator(13)
        # self.elevatorPid = TrapezoidProfile(TrapezoidProfile.Constraints(1, 1))

        # self.elevator.setSmartDashboard(SendableRegistry.setName)
        SendableRegistry.setName(self.elevator.elevatorPid, "Elevator")

        # Shoulder
        self.shoulder = Components.Shoulder(16)

        # # Intake
        self.intake = Components.Intake(14)

        self.wrist = Components.Wrist(
            11,
            # 2 * pi
        )

        # CLimber
        self.climb = SparkMax(10, SparkMax.MotorType.kBrushless)
        self.climbEncoder = self.climb.getEncoder()

        #################################################################################################################################

        self.timer = wpilib.Timer()

        self.timer.start()

        wpilib.CameraServer.launch()

        remoteRIODebugSupport()

        self.crashLogger = CrashLogger()

        # We do our own logging, we don't need additional logging in the background.
        # Both of these will increase CPU load by a lot, and we never use their output.
        wpilib.LiveWindow.disableAllTelemetry()

        self.webserver = Webserver()

        self.driveTrain = DrivetrainControl()
        self.autodrive = AutoDrive()

        self.stt = SegmentTimeTracker()

        self.dInt = DriverInterface()
        self.stick = self.dInt.ctrl
        self.stick2 = wpilib.XboxController(1)

        self.autoSequencer = AutoSequencer()

        self.dashboard = Dashboard()

        self.rioMonitor = RIOMonitor()
        self.pwrMon = PowerMonitor()

        # Normal robot code updates every 20ms, but not everything needs to be that fast.
        # Register slower-update periodic functions
        self.addPeriodic(self.pwrMon.update, 0.2, 0.0)
        self.addPeriodic(self.crashLogger.update, 1.0, 0.0)
        self.addPeriodic(CalibrationWrangler().update, 0.5, 0.0)
        self.addPeriodic(FaultWrangler().update, 0.2, 0.0)

        self.autoHasRun = False

    def robotPeriodic(self):
        self.led.setData(self.ledData)

        # print("Elevator", self.elevator.getPosition())
        print("Wrist", self.wrist.getPosition())
        print("errorWrist", self.wrist.wristPid.getError())
        print("wristSetpoint", self.wristSetpoint)
        # print("Shoulder", self.shoulder.getPosition())
        # print("ShoulderError", self.shoulder.shoulderPid.getError())
        print("STATE", self.state)

        if self.limitSwitch.get() == False:
            self.elevator.elevatorEncoder.setPosition(0)
            self.color(235, 140, 73)

        #########################################################

        self.stt.start()

        self.dInt.update()
        self.stt.mark("Driver Interface")

        self.driveTrain.update()
        self.stt.mark("Drivetrain")

        self.autodrive.updateTelemetry()
        self.driveTrain.poseEst._telemetry.setCurAutoDriveWaypoints(
            self.autodrive.getWaypoints()
        )
        self.driveTrain.poseEst._telemetry.setCurObstacles(
            self.autodrive.rfp.getObstacleStrengths()
        )
        self.stt.mark("Telemetry")

        logUpdate()
        self.stt.end()

    #########################################################

    def autonomousInit(self):
        # Start up the autonomous sequencer
        self.autoSequencer.initialize()

        # Use the autonomous routines starting pose to init the pose estimator
        self.driveTrain.poseEst.setKnownPose(self.autoSequencer.getStartingPose())

        # Mark we at least started autonomous
        self.autoHasRun = True

        self.timer.restart()

    def autonomousPeriodic(self):
        self.autoSequencer.update()

        # Operators cannot control in autonomous
        self.driveTrain.setManualCmd(DrivetrainCommand())

        if self.timer == 0.7:
            self.color(0, 255, 0)

    def autonomousExit(self):
        self.autoSequencer.end()

    #########################################################
    ## Teleop-Specific init and update
    def teleopInit(self):
        # clear existing telemetry trajectory
        self.driveTrain.poseEst._telemetry.setCurAutoTrajectory(None)

        # If we're starting teleop but haven't run auto, set a nominal default pose
        # This is needed because initial pose is usually set by the autonomous routine
        if not self.autoHasRun:
            self.driveTrain.poseEst.setKnownPose(Pose2d(1.0, 1.0, Rotation2d(0.0)))

        self.timer.restart()

    def teleopPeriodic(self):
        # TODO - this is technically one loop delayed, which could induce lag
        # Probably not noticeable, but should be corrected.
        # self.driveTrain.setManualCmd(self.dInt.getCmd())
        # print("Tx:", self.tx.getDouble(0))
        # print("Elevator", self.elevator.getPosition())
        # print("Wrist", self.wrist.getPosition())
        # print("Shoulder", self.shoulder.getPosition())
        # print("time", self.timer.get())
        # print("WristError", self.wrist.wristPid.getError())
        # print("Wrist", self.wrist.getPosition())
        print("Climb", self.climbEncoder.getPosition())

        if self.timer.get() >= 125:
            self.color(160, 100, 100)

        # # COLOR

        # CLIMBER
        if self.stick.getRawButton(3):
            self.climb.set(1)
            self.rainbow()
        elif self.stick.getRawButton(4):
            self.climb.set(-1)
            self.color(255, 0, 0)
        else:
            self.climb.set(0)

        # INTAKE
        if self.stick2.getRightTriggerAxis() > 0:
            self.intake.set(-1)
        elif self.stick2.getLeftTriggerAxis() > 0:
            self.intake.set(1)
        else:
            self.intake.set(self.intake.intakeFeed.calculate(0.1))

        # # SHOULDER
        # if self.stick2.getRawButton(5):
        #     self.shoulder.set(-0.3)
        # elif self.stick2.getRawButton(6):
        #     self.shoulder.set(0.3)
        # else:
        #     self.shoulder.set(0)

        # # SHOULDER PID
        if self.stick2.getRawButton(5):
            self.shoulder.set(
                self.shoulder.calculate(self.shoulder.getPosition(), 0.122)
            )
        elif self.stick2.getRawButton(6):
            self.shoulder.set(
                self.shoulder.calculate(self.shoulder.getPosition(), 0.0048)
            )
        else:
            self.shoulder.set(0)

        # TRAPEZOIDAL ELEVATOR
        if self.stick2.getPOV() == 0:
            self.elevator.set(-0.3)
            self.color(0, 0, 255)
        elif self.stick2.getPOV() == 180:
            self.elevator.set(0.3)
            self.color(0, 0, 50)
        # elif self.stick.getPOV() == 0:
        #     self.elevator.set(
        #         self.elevator.calculate(self.elevator.getPosition(), -3)
        #         + -self.elevator.elevatorFeedForward.calculate(0.3)
        #     )
        #     self.color(0, 0, 255)
        # elif self.stick.getPOV() == 180:
        #     self.elevator.set(
        #         self.elevator.calculate(self.elevator.getPosition(), 0)
        #         + -self.elevator.elevatorFeedForward.calculate(0.3)
        #     )
        #     self.color(0, 0, 50)
        # else:
        #     self.elevator.set(-self.elevator.elevatorFeedForward.calculate(0))

        # self.wrist.set(
        #     self.wrist.calculate(self.wrist.getPosition(), self.wristSetpoint)
        # )

        # if self.stick2.getRawButton(1):
        #     if self.state == True:
        #         self.wristSetpoint = 0.52
        #         self.state = False
        #         self.color(0, 0, 255)
        #         time.sleep(0.1)
        #     elif self.state == False:
        #         self.wristSetpoint = 0.267
        #         self.state = True
        #         self.color(0, 255, 0)
        #         time.sleep(0.1)

        # LIMELIGHT ALIGN
        if self.stick.getRawButton(10) and self.tx.getDouble(0) == range(10, 15):
            print("right")
            self.driveTrain.setManualCmd(DrivetrainCommand(0, -0.3, 0))
            self.color(0, 255, 30)
        elif self.stick.getRawButton(10) and self.tx.getDouble(0) >= 15.001:
            print("right")
            self.driveTrain.setManualCmd(DrivetrainCommand(0, -0.6, 0))
            self.color(0, 255, 30)
        elif self.stick.getRawButton(10) and self.tx.getDouble(0) == range(-10, -15):
            print("left")
            self.driveTrain.setManualCmd(DrivetrainCommand(0, 0.3, 0))
            self.color(30, 255, 0)
        elif self.stick.getRawButton(10) and self.tx.getDouble(0) <= -15.001:
            print("left")
            self.driveTrain.setManualCmd(DrivetrainCommand(0, 0.6, 0))
            self.color(30, 255, 0)
        else:
            self.driveTrain.setManualCmd(self.dInt.getCmd())

        # if self.dInt.getGyroResetCmd():
        #     self.driveTrain.resetGyro()

        # if self.dInt.getCreateObstacle():
        #     # For test purposes, inject a series of obstacles around the current pose
        #     ct = self.driveTrain.poseEst.getCurEstPose().translation()
        #     tfs = [
        #         # Translation2d(1.7, -0.5),
        #         # Translation2d(0.75, -0.75),
        #         # Translation2d(1.7, 0.5),
        #         Translation2d(0.75, 0.75),
        #         Translation2d(2.0, 0.0),
        #         Translation2d(0.0, 1.0),
        #         Translation2d(0.0, -1.0),
        #     ]
        #     for tf in tfs:
        #         obs = PointObstacle(location=(ct + tf), strength=0.5)
        #         self.autodrive.rfp.addObstacleObservation(obs)

        # self.autodrive.setRequest(self.dInt.getAutoDrive())

        # No trajectory in Teleop
        Trajectory().setCmd(None)

    #########################################################
    ## Disabled-Specific init and update
    def disabledPeriodic(self):
        self.autoSequencer.updateMode()
        Trajectory().trajHDC.updateCals()

    def disabledInit(self):
        self.autoSequencer.updateMode(True)

    #########################################################
    ## Test-Specific init and update
    def testInit(self):
        # wpilib.LiveWindow.setEnabled(False)
        pass

    def testPeriodic(self):
        pass

    #########################################################
    ## Cleanup
    def endCompetition(self):
        # Sometimes `robopy test pyfrc_test.py` will invoke endCompetition() without completing robotInit(),
        # this will create a confusing exception here because we can reach self.rioMonitor.stopThreads()
        # when self.rioMonitor does not exist.
        # To prevent the exception and confusion, we only call self.rioMonitor.stopThreads() when exists.
        rioMonitorExists = getattr(self, "rioMonitor", None)
        if rioMonitorExists is not None:
            self.rioMonitor.stopThreads()

        destroyAllSingletonInstances()
        super().endCompetition()

    def rainbow(self):
        for i in range(kLEDBuffer):
            hue = (self.rainbowFirstPixelHue + (i * 180 / kLEDBuffer)) % 180
            self.ledData[i].setHSV(int(hue), 255, 128)
        self.rainbowFirstPixelHue += 3
        self.rainbowFirstPixelHue %= 180

    def color(self, R: int, G: int, B: int):
        for i in range(kLEDBuffer):
            hue = (self.rainbowFirstPixelHue + (i * 180 / kLEDBuffer)) % 180
            self.ledData[i].setRGB(R, G, B)
        self.rainbowFirstPixelHue += 3
        self.rainbowFirstPixelHue %= 180


def remoteRIODebugSupport():
    if __debug__ and "run" in sys.argv:
        print("Starting Remote Debug Support....")
        try:
            import debugpy  # pylint: disable=import-outside-toplevel
        except ModuleNotFoundError:
            pass
        else:
            debugpy.listen(("0.0.0.0", 5678))
            debugpy.wait_for_client()
