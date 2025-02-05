import sys
from phoenix6 import SignalLogger
import wpilib
from wpilib import AddressableLED, DigitalInput, LEDPattern, Color
from dashboard import Dashboard
from drivetrain.controlStrategies.autoDrive import AutoDrive
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainControl import DrivetrainControl
from humanInterface.driverInterface import DriverInterface
from subsystems import Components, limelight
from rev import SparkMax

# from humanInterface.ledControl import LEDControl
from navigation.forceGenerators import PointObstacle
from utils.segmentTimeTracker import SegmentTimeTracker
from utils.signalLogging import logUpdate
from utils.calibration import CalibrationWrangler
from utils.faults import FaultWrangler
from utils.crashLogger import CrashLogger
from utils.rioMonitor import RIOMonitor
from utils.singleton import destroyAllSingletonInstances
from utils.powerMonitor import PowerMonitor
from webserver.webserver import Webserver
from AutoSequencerV2.autoSequencer import AutoSequencer
from utils.powerMonitor import PowerMonitor
from wpimath.geometry import Translation2d, Pose2d, Rotation2d

kLEDBuffer = 150


class MyRobot(wpilib.TimedRobot):
    #########################################################
    ## Common init/update for all modes
    def robotInit(self):
        # Since we're defining a bunch of new things here, tell pylint
        # to ignore these instantiations in a method.
        # pylint: disable=attribute-defined-outside-init
        # LEDS
        self.led = wpilib.AddressableLED(8)
        self.ledData = [wpilib.AddressableLED.LEDData() for _ in range(kLEDBuffer)]
        self.rainbowFirstPixelHue = 0
        self.led.setLength(kLEDBuffer)
        self.led.setData(self.ledData)
        self.led.start()

        self.limitSwitch = DigitalInput(7)

        # Elevator
        self.elevator = Components.Elevator(10)

        # Shoulder
        self.shoulder = Components.Shoulder(11)

        # # Intake
        self.intake = Components.Intake(12)

        self.wrist = Components.Wrist(13)

        # CLimber
        self.climb = SparkMax(14, SparkMax.MotorType.kBrushless)

        #################################################################################################################################

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
        # self.color(3, 38, 252)
        self.led.setData(self.ledData)

        # COLOR
        if self.limitSwitch.get() == True:
            self.color(0, 0, 255)  # Blue
        if self.limitSwitch.get() == False:
            self.color(255, 0, 0)  # Red

        # CLIMBER
        if self.stick.getLeftBumper():
            self.climb.set(1)
            self.rainbow()
        elif self.stick.getRightBumper():
            self.climb.set(-1)
        else:
            self.climb.set(0)

        # INTAKE
        if self.stick.getRawButton(8):
            self.intake.set(-0.6)
        elif self.stick.getRawButton(7):
            self.intake.set(0.6)
        else:
            self.intake.set(0)

        # SHOULDER
        if self.stick.getRawButton(1):
            self.shoulder.set(-0.3)
        elif self.stick.getRawButton(4):
            self.shoulder.set(0.3)
        else:
            self.shoulder.set(0)

        # ELEVATOR
        if self.stick.getRawButton(3):
            self.elevator.set(-0.3)
        elif self.stick.getRawButton(2):
            self.elevator.set(0.3)
        else:
            self.elevator.set(-self.elevator.elevatorFeedForward.calculate(0.01))

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

        # Use the autonomous rouines starting pose to init the pose estimator
        self.driveTrain.poseEst.setKnownPose(self.autoSequencer.getStartingPose())

        # Mark we at least started autonomous
        self.autoHasRun = True

    def autonomousPeriodic(self):
        self.autoSequencer.update()

        # Operators cannot control in autonomous
        self.driveTrain.setManualCmd(DrivetrainCommand())

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

    def teleopPeriodic(self):
        # TODO - this is technically one loop delayed, which could induce lag
        # Probably not noticeable, but should be corrected.
        print(self.limitSwitch.get())
        # if self.limitSwitch.get() == True:
        #     self.led.setData(kLEDBuffer)

        # if self.limitSwitch.get() == False:
        #     self.led.setData(kLEDBuffer)

        if self.dInt.getGyroResetCmd():
            self.driveTrain.resetGyro()

        if self.dInt.getCreateObstacle():
            # For test purposes, inject a series of obstacles around the current pose
            ct = self.driveTrain.poseEst.getCurEstPose().translation()
            tfs = [
                # Translation2d(1.7, -0.5),
                # Translation2d(0.75, -0.75),
                # Translation2d(1.7, 0.5),
                Translation2d(0.75, 0.75),
                Translation2d(2.0, 0.0),
                Translation2d(0.0, 1.0),
                Translation2d(0.0, -1.0),
            ]
            for tf in tfs:
                obs = PointObstacle(location=(ct + tf), strength=0.5)
                self.autodrive.rfp.addObstacleObservation(obs)

        self.autodrive.setRequest(self.dInt.getAutoDrive())

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
        wpilib.LiveWindow.setEnabled(False)

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
