import sys
import time
from math import pi, sqrt

from drivetrain.poseEstimation.drivetrainPoseEstimator import DrivetrainPoseEstimator
import wpilib
from networktables import NetworkTables
from phoenix6 import SignalLogger
from rev import SparkMax
from wpilib import AddressableLED, Color, DigitalInput, LEDPattern, LiveWindow
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrapezoidProfile
from wpiutil import SendableRegistry
from wpimath.controller import PIDController

from AutoSequencerV2.autoSequencer import AutoSequencer
from dashboard import Dashboard
from drivetrain.controlStrategies.autoDrive import AutoDrive
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainControl import DrivetrainControl
from drivetrain.poseEstimation.drivetrainPoseEstimator import DrivetrainPoseEstimator
from humanInterface.driverInterface import DriverInterface

# from humanInterface.ledControl import LEDControl
from navigation.forceGenerators import PointObstacle
from subsystems import Components, limelight

# from subsystems.presets import Presets
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
        self.wristSetpoint = 0.447

        # LEDS
        self.led = wpilib.AddressableLED(0)
        self.ledData = [wpilib.AddressableLED.LEDData() for _ in range(kLEDBuffer)]
        self.rainbowFirstPixelHue = 0
        self.led.setLength(kLEDBuffer)
        self.led.setData(self.ledData)
        self.led.start()

        self.limitSwitch = DigitalInput(7)

        # Limelight
        NetworkTables.initialize(server="10.67.58.3")
        self.limeLight2 = NetworkTables.getTable("limelight-kmrobob")
        self.Priority2 = self.limeLight2.getEntry("tid")

        NetworkTables.initialize(server="10.67.58.2")
        self.limeLight = NetworkTables.getTable("limelight-kmrobot")
        # self.tx = self.limeLight.getEntry("tx").getDouble(0)
        # self.ty = self.limeLight.getEntry("ty").getDouble(0)
        self.Priority = self.limeLight.getEntry("tid")

        # Elevator
        self.elevator = Components.Elevator(13)

        self.elevator.setSmartDashboard(SendableRegistry.setName)
        SendableRegistry.setName(self.elevator.elevatorPid, "Elevator")

        # Shoulder
        self.shoulder = Components.Shoulder(16)

        # # Intake
        self.intake = Components.Intake(14)

        self.wrist = Components.Wrist(
            11,
        )
        #     # 2 * pi
        # )

        # CLimber
        self.climb = Components.Climb(10)

        #########################################################

        self.drivePid = PIDController(2.5, 0, 0.02, 0.01)
        self.drivePidX = PIDController(0.04, 0, 0.004, 0.01)
        self.drivePidY = PIDController(0.08, 0, 0.008, 0.01)

        self.AutodrivePid = PIDController(0.5, 0, 0.02, 0.01)
        self.AutodrivePidX = PIDController(0.03, 0, 0.003, 0.01)
        self.AutodrivePidY = PIDController(0.04, 0, 0.004, 0.01)

        self.leftdrivePid = PIDController(1, 0, 0.02, 0.01)
        self.leftdrivePidX = PIDController(0.03, 0, 0.003, 0.01)
        self.leftdrivePidY = PIDController(0.04, 0, 0.004, 0.01)

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

        self.autoHasRun = False

    def robotPeriodic(self):
        # print("Elevator", self.elevator.getPosition())
        # print("Wrist", self.wrist.getPosition())
        # print("Shoulder", self.shoulder.getPosition())
        # # print("WristSetpoint", self.wrist.wristPid.getSetpoint())
        print("TX", self.limeLight.getEntry("tx").getDouble(0))
        print("Ty", self.limeLight.getEntry("ty").getDouble(0))
        print("TX2", self.limeLight2.getEntry("tx").getDouble(0))
        print("Ty2", self.limeLight2.getEntry("ty").getDouble(0))
        print("ID", int(self.Priority.getDouble(0)))
        print("ID2", int(self.Priority.getDouble(0)))
        print("POSE", self.driveTrain.poseEst.getCurEstPose().rotation().radians())
        # print("ClimbEnc", self.climb.getPosition())
        # print("timer", self.timer.get())

        #########################################################

        self.led.setData(self.ledData)

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

        print("timer", self.timer.get())

        # Operators cannot control in autonomous
        self.driveTrain.setManualCmd(DrivetrainCommand())

        if self.timer.get() >= 6 and self.timer.get() <= 8:
            print("RANGE")
            self.shoulder.set(
                self.shoulder.shoulderPid.calculate(
                    self.shoulder.shoulderEncoder.getOutput(), 0.226
                )
            )
            self.elevator.set(
                -self.elevator.calculate(self.elevator.getPosition(), -6.22)
            )

        if self.timer.get() >= 8 and self.timer.get() <= 8.5:
            self.shoulder.set(
                self.shoulder.shoulderPid.calculate(
                    self.shoulder.shoulderEncoder.getOutput(), 0.359
                )
            )
            self.intake.set(0.3)

        if self.timer.get() >= 8.6 and self.timer.get() <= 9:
            self.shoulder.set(
                self.shoulder.shoulderPid.calculate(
                    self.shoulder.shoulderEncoder.getOutput(), 0.226
                )
            )

        # CHANGE PRIORITY TO 20 AT COMP
        if (
            self.timer.get() >= 2
            and self.timer.get() <= 8
            and (int(self.Priority.getDouble(0) == 7))
        ):
            self.driveTrain.setManualCmd(
                DrivetrainCommand(
                    self.AutodrivePidY.calculate(
                        self.limeLight.getEntry("ty").getDouble(0), 8.37
                    ),
                    self.AutodrivePidX.calculate(
                        self.limeLight.getEntry("tx").getDouble(0), -10.6
                    ),
                    self.AutodrivePid.calculate(
                        self.driveTrain.poseEst.getCurEstPose().rotation().radians(),
                        1.032,
                    ),
                )
            )

        if self.timer.get() >= 2.2:
            self.autoSequencer.end()

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

        if self.timer.get() >= 125:
            self.color(255, 0, 0)
            # self.climb.set(self.climb.calculate(self.climb.getPosition, value))

        # CLIMBER
        if self.stick.getRawButton(3):
            self.climb.set(1)
            self.rainbow()
        elif self.stick.getRawButton(4):
            self.climb.set(-1)
            self.color(255, 0, 0)
        else:
            self.climb.set(0)

        #########################################################

        # WRIST PID SET
        self.wrist.set(
            self.wrist.calculate(self.wrist.getPosition(), self.wristSetpoint)
        )

        if self.stick2.getRawButton(5):
            self.shoulder.set(0.3)

        elif self.stick2.getRawButton(6):
            self.shoulder.set(-0.3)

        elif self.stick2.getPOV() == 0:
            self.elevator.set(0.3)

        elif self.stick2.getPOV() == 180:
            self.elevator.set(-0.3)

        # CORAL STATION PRESET
        elif self.stick.getRawButton(6):
            self.shoulder.set(
                self.shoulder.shoulderPid.calculate(
                    self.shoulder.shoulderEncoder.getOutput(), 0.192
                )
            )
            self.elevator.set(
                -self.elevator.calculate(self.elevator.getPosition(), -0.57)
            )
            self.wristSetpoint = 0.70
            self.color(100, 0, 145)

        # TROUGH PRESET
        elif self.stick.getPOV() == 180:
            self.shoulder.set(
                self.shoulder.shoulderPid.calculate(
                    self.shoulder.shoulderEncoder.getOutput(), 0.42
                )
            )
            self.elevator.set(
                -self.elevator.calculate(self.elevator.getPosition(), -1.95)
            )
            self.blueGradient()

        # LEVEL TWO - L2
        elif self.stick.getPOV() == 90:
            self.shoulder.set(
                self.shoulder.shoulderPid.calculate(
                    self.shoulder.shoulderEncoder.getOutput(), 0.259
                )
            )
            self.elevator.set(
                -self.elevator.calculate(self.elevator.getPosition(), -1.21)
            )
            self.blueGradient()

        # LEVEL THREE - L3
        elif self.stick.getPOV() == 0:
            self.shoulder.set(
                self.shoulder.shoulderPid.calculate(
                    self.shoulder.shoulderEncoder.getOutput(), 0.289
                )
            )
            self.elevator.set(
                -self.elevator.calculate(self.elevator.getPosition(), -3.29)
            )
            self.blueGradient()

        # LEVEL FOUR - L4
        elif self.stick.getPOV() == 270:
            self.shoulder.set(
                self.shoulder.shoulderPid.calculate(
                    self.shoulder.shoulderEncoder.getOutput(), 0.226
                )
            )
            self.elevator.set(
                -self.elevator.calculate(self.elevator.getPosition(), -6.22)
            )
            self.blueGradient()

        # BARGE PRESET - 0.379 - scoring preset
        elif self.stick.getRawButton(8):
            self.shoulder.set(
                self.shoulder.shoulderPid.calculate(
                    self.shoulder.shoulderEncoder.getOutput(), 0.255
                )
            )
            self.elevator.set(
                -self.elevator.calculate(self.elevator.getPosition(), -7.73)
            )
            self.blueGradient()

        # INTAKE
        elif self.stick.getRightTriggerAxis() > 0:
            self.intake.set(-1)
        elif self.stick.getLeftTriggerAxis() > 0:
            self.intake.set(1)

        # SCORING PRESET (SHOULDER + INTAKE)
        elif self.stick2.getRightTriggerAxis() > 0:
            self.shoulder.set(
                self.shoulder.shoulderPid.calculate(
                    self.shoulder.shoulderEncoder.getOutput(), 0.359
                )
            )
            self.intake.set(0.5)

        # SAFETY DRIVING PRESET
        elif self.stick.getRawButton(5):
            self.shoulder.set(
                self.shoulder.shoulderPid.calculate(
                    self.shoulder.shoulderEncoder.getOutput(), 0.225
                )
            )
            self.elevator.set(
                -self.elevator.calculate(self.elevator.getPosition(), -0.5)
            )
            self.wristSetpoint = 0.447

        else:
            self.elevator.set(self.elevator.elevatorFeedForward.calculate(0))
            self.shoulder.set(0)
            self.intake.set(-self.intake.intakeFeed.calculate(0.1))

        # wrist toggle - 0.37
        if self.stick2.getRawButton(1):
            if self.state == True:
                self.wristSetpoint = 0.20
                self.state = False
                self.color(255, 0, 255)
                time.sleep(0.1)

            elif self.state == False:
                self.wristSetpoint = 0.447
                self.state = True
                self.color(0, 255, 0)
                time.sleep(0.1)

        # LIMELIGHT ALIGN (Left Side Values: TX: 27.58 TY: 5.74) (Right Side Values: TX: -10.6 TY: 8.37)
        if (
            self.stick.getRawButton(9)
            and (int(self.Priority.getDouble(0) == 7))
            or (int(self.Priority.getDouble(0) == 18))
        ):
            self.driveTrain.setManualCmd(
                DrivetrainCommand(
                    self.drivePidY.calculate(
                        self.limeLight.getEntry("ty").getDouble(0), 5.74
                    ),
                    self.drivePidX.calculate(
                        self.limeLight.getEntry("tx").getDouble(0), 27.58
                    ),
                    self.drivePid.calculate(
                        self.driveTrain.poseEst.getCurEstPose().rotation().radians(),
                        0,
                    ),
                )
            )
        elif (
            self.stick.getRawButton(9)
            and (int(self.Priority.getDouble(0) == 8))
            or (int(self.Priority.getDouble(0) == 17))
        ):
            self.driveTrain.setManualCmd(
                DrivetrainCommand(
                    self.drivePidY.calculate(
                        self.limeLight.getEntry("ty").getDouble(0), 5.74
                    ),
                    self.drivePidX.calculate(
                        self.limeLight.getEntry("tx").getDouble(0), 27.58
                    ),
                    self.drivePid.calculate(
                        self.driveTrain.poseEst.getCurEstPose().rotation().radians(),
                        1,
                    ),
                )
            )
        elif (
            self.stick.getRawButton(9)
            and (int(self.Priority.getDouble(0) == 9))
            or (int(self.Priority.getDouble(0) == 22))
        ):
            self.driveTrain.setManualCmd(
                DrivetrainCommand(
                    self.drivePidY.calculate(
                        self.limeLight.getEntry("ty").getDouble(0), 5.74
                    ),
                    self.drivePidX.calculate(
                        self.limeLight.getEntry("tx").getDouble(0), 27.58
                    ),
                    self.drivePid.calculate(
                        self.driveTrain.poseEst.getCurEstPose().rotation().radians(),
                        2.104,
                    ),
                )
            )
        elif (
            self.stick.getRawButton(9)
            and (int(self.Priority.getDouble(0) == 10))
            or (int(self.Priority.getDouble(0) == 21))
        ):
            self.driveTrain.setManualCmd(
                DrivetrainCommand(
                    self.drivePidY.calculate(
                        self.limeLight.getEntry("ty").getDouble(0), 5.74
                    ),
                    self.drivePidX.calculate(
                        self.limeLight.getEntry("tx").getDouble(0), 27.58
                    ),
                    self.drivePid.calculate(
                        self.driveTrain.poseEst.getCurEstPose().rotation().radians(),
                        -3.123,
                    ),
                )
            )
        elif (
            self.stick.getRawButton(9)
            and (int(self.Priority.getDouble(0) == 11))
            or (int(self.Priority.getDouble(0) == 20))
        ):
            self.driveTrain.setManualCmd(
                DrivetrainCommand(
                    self.drivePidY.calculate(
                        self.limeLight.getEntry("ty").getDouble(0), 5.74
                    ),
                    self.drivePidX.calculate(
                        self.limeLight.getEntry("tx").getDouble(0), 27.58
                    ),
                    self.drivePid.calculate(
                        self.driveTrain.poseEst.getCurEstPose().rotation().radians(),
                        -1.98,
                    ),
                )
            )
        elif (
            self.stick.getRawButton(9)
            and (int(self.Priority.getDouble(0) == 6))
            or (int(self.Priority.getDouble(0) == 19))
        ):
            self.driveTrain.setManualCmd(
                DrivetrainCommand(
                    self.drivePidY.calculate(
                        self.limeLight.getEntry("ty").getDouble(0), 5.74
                    ),
                    self.drivePidX.calculate(
                        self.limeLight.getEntry("tx").getDouble(0), 27.58
                    ),
                    self.drivePid.calculate(
                        self.driveTrain.poseEst.getCurEstPose().rotation().radians(),
                        -1.12,
                    ),
                )
            )
        # LIMELIGHT ALIGN (Left Side Values: TX: 27.58 TY: 5.74) (Right Side Values: TX: -10.6 TY: 8.37)
        elif (
            self.stick.getRawButton(10)
            and (int(self.Priority.getDouble(0) == 7))
            or (int(self.Priority.getDouble(0) == 18))
        ):
            self.driveTrain.setManualCmd(
                DrivetrainCommand(
                    self.drivePidY.calculate(
                        self.limeLight.getEntry("ty").getDouble(0), 8.37
                    ),
                    self.drivePidX.calculate(
                        self.limeLight.getEntry("tx").getDouble(0), -10.6
                    ),
                    self.drivePid.calculate(
                        self.driveTrain.poseEst.getCurEstPose().rotation().radians(),
                        0,
                    ),
                )
            )
        elif (
            self.stick.getRawButton(10)
            and (int(self.Priority.getDouble(0) == 8))
            or (int(self.Priority.getDouble(0) == 17))
        ):
            self.driveTrain.setManualCmd(
                DrivetrainCommand(
                    self.drivePidY.calculate(
                        self.limeLight.getEntry("ty").getDouble(0), 8.37
                    ),
                    self.drivePidX.calculate(
                        self.limeLight.getEntry("tx").getDouble(0), -10.6
                    ),
                    self.drivePid.calculate(
                        self.driveTrain.poseEst.getCurEstPose().rotation().radians(),
                        1,
                    ),
                )
            )
        elif (
            self.stick.getRawButton(10)
            and (int(self.Priority.getDouble(0) == 9))
            or (int(self.Priority.getDouble(0) == 22))
        ):
            self.driveTrain.setManualCmd(
                DrivetrainCommand(
                    self.drivePidY.calculate(
                        self.limeLight.getEntry("ty").getDouble(0), 8.37
                    ),
                    self.drivePidX.calculate(
                        self.limeLight.getEntry("tx").getDouble(0), -10.6
                    ),
                    self.drivePid.calculate(
                        self.driveTrain.poseEst.getCurEstPose().rotation().radians(),
                        2.104,
                    ),
                )
            )
        elif (
            self.stick.getRawButton(10)
            and (int(self.Priority.getDouble(0) == 10))
            or (int(self.Priority.getDouble(0) == 21))
        ):
            self.driveTrain.setManualCmd(
                DrivetrainCommand(
                    self.drivePidY.calculate(
                        self.limeLight.getEntry("ty").getDouble(0), 8.37
                    ),
                    self.drivePidX.calculate(
                        self.limeLight.getEntry("tx").getDouble(0), -10.6
                    ),
                    self.drivePid.calculate(
                        self.driveTrain.poseEst.getCurEstPose().rotation().radians(),
                        -3.123,
                    ),
                )
            )
        elif (
            self.stick.getRawButton(10)
            and (int(self.Priority.getDouble(0) == 11))
            or (int(self.Priority.getDouble(0) == 20))
        ):
            self.driveTrain.setManualCmd(
                DrivetrainCommand(
                    self.drivePidY.calculate(
                        self.limeLight.getEntry("ty").getDouble(0), 8.37
                    ),
                    self.drivePidX.calculate(
                        self.limeLight.getEntry("tx").getDouble(0), -10.6
                    ),
                    self.drivePid.calculate(
                        self.driveTrain.poseEst.getCurEstPose().rotation().radians(),
                        -2.03,
                    ),
                )
            )
        elif (
            self.stick.getRawButton(10)
            and (int(self.Priority.getDouble(0) == 6))
            or (int(self.Priority.getDouble(0) == 19))
        ):
            self.driveTrain.setManualCmd(
                DrivetrainCommand(
                    self.drivePidY.calculate(
                        self.limeLight.getEntry("ty").getDouble(0), 8.37
                    ),
                    self.drivePidX.calculate(
                        self.limeLight.getEntry("tx").getDouble(0), -10.6
                    ),
                    self.drivePid.calculate(
                        self.driveTrain.poseEst.getCurEstPose().rotation().radians(),
                        -1.12,
                    ),
                )
            )
        else:
            self.driveTrain.setManualCmd(self.dInt.getCmd())

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

    def blueGradient(self):
        for i in range(kLEDBuffer):
            hue = self.elevator.getPosition() * -33
            self.ledData[i].setRGB(0, 0, int(hue))

    def endLights(self):
        for i in range(kLEDBuffer):
            hue = self.timer.get() * 1.3
            self.ledData[i].setRGB(int(hue), int(hue), int(hue))


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
