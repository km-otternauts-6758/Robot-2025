from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import (
    MAX_FWD_REV_SPEED_MPS,
    MAX_STRAFE_SPEED_MPS,
    MAX_ROTATE_SPEED_RAD_PER_SEC,
    MAX_TRANSLATE_ACCEL_MPS2,
    MAX_ROTATE_ACCEL_RAD_PER_SEC_2,
)
from utils.allianceTransformUtils import onRed
from utils.faults import Fault
from utils.signalLogging import addLog
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpilib import XboxController
from subsystems.Components import Shoulder, Wrist, Elevator, Intake, Climb
import enum

kElevatorButton = 0


# class ElevatorButton(enum.Enum):
#     NONE = 0
#     L1 = 1
#     L2 = 2
#     L3 = 3
#     L4 = 4


class DriverInterface:
    """Class to gather input from the driver of the robot"""

    def __init__(self):
        # controller
        ctrlIdx = 0
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Driver XBox controller ({ctrlIdx}) unplugged")

        # Drivetrain motion commands
        self.velXCmd = 0
        self.velYCmd = 0
        self.velTCmd = 0

        # Driver motion rate limiters - enforce smoother driving
        self.velXSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_TRANSLATE_ACCEL_MPS2)
        self.velYSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_TRANSLATE_ACCEL_MPS2)
        self.velTSlewRateLimiter = SlewRateLimiter(
            rateLimit=MAX_ROTATE_ACCEL_RAD_PER_SEC_2
        )

        # Navigation commands
        self.autoDrive = False
        self.createDebugObstacle = False

        # Utility - reset to zero-angle at the current pose
        self.gyroResetCmd = False

        # Logging
        # addLog("DI FwdRev Cmd", lambda: self.velXCmd, "mps")
        # addLog("DI Strafe Cmd", lambda: self.velYCmd, "mps")
        # addLog("DI Rot Cmd", lambda: self.velTCmd, "radps")
        # addLog("DI gyroResetCmd", lambda: self.gyroResetCmd, "bool")
        # addLog("DI autoDriveToSpeaker", lambda: self.autoDriveToSpeaker, "bool")
        # addLog("DI autoDriveToPickup", lambda: self.autoDriveToPickup, "bool")

    def update(self):
        # value of contoller buttons

        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions
            vXJoyRaw = self.ctrl.getLeftY() * 1
            vYJoyRaw = self.ctrl.getLeftX() * 1
            vRotJoyRaw = self.ctrl.getRightX() * -1

            # Correct for alliance
            if onRed():
                vXJoyRaw *= -1.0
                vYJoyRaw *= -1.0

            # deadband
            vXJoyWithDeadband = applyDeadband(vXJoyRaw, 0.2)
            vYJoyWithDeadband = applyDeadband(vYJoyRaw, 0.2)
            vRotJoyWithDeadband = applyDeadband(vRotJoyRaw, 0.2)

            # TODO - if the driver wants a slow or sprint button, add it here.
            slowMult = 1.0 if (self.ctrl.getRightBumper()) else 0.7
            # slowMult = 1.0

            # Shape velocity command
            velCmdXRaw = vXJoyWithDeadband * MAX_STRAFE_SPEED_MPS * slowMult
            velCmdYRaw = vYJoyWithDeadband * MAX_FWD_REV_SPEED_MPS * slowMult
            velCmdRotRaw = vRotJoyWithDeadband * MAX_ROTATE_SPEED_RAD_PER_SEC * 0.8

            # Slew rate limiter
            self.velXCmd = self.velXSlewRateLimiter.calculate(velCmdXRaw)
            self.velYCmd = self.velYSlewRateLimiter.calculate(velCmdYRaw)
            self.velTCmd = self.velTSlewRateLimiter.calculate(velCmdRotRaw)

            self.gyroResetCmd = self.ctrl.getAButton()

            self.autoDrive = self.ctrl.getBButton()
            self.createDebugObstacle = self.ctrl.getYButtonPressed()

            self.connectedFault.setNoFault()

        else:
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            self.velXCmd = 0.0
            self.velYCmd = 0.0
            self.velTCmd = 0.0
            self.gyroResetCmd = False
            self.autoDrive = False
            self.createDebugObstacle = False
            self.connectedFault.setFaulted()

    def getCmd(self) -> DrivetrainCommand:
        retval = DrivetrainCommand()
        retval.velX = self.velXCmd
        retval.velY = self.velYCmd
        retval.velT = self.velTCmd
        return retval

    # def getAutoDrive(self) -> bool:
    #     return self.autoDrive

    # def getGyroResetCmd(self) -> bool:
    #     return self.gyroResetCmd

    # def getCreateObstacle(self) -> bool:
    #     return self.createDebugObstacle

    # def getElevator(self) -> ElevatorButton:
    #     if (
    #         316 < self.ctrl.getPOV(kElevatorButton) < 359
    #         and 0 < self.ctrl.getPOV(kElevatorButton) < 45
    #     ):
    #         return ElevatorButton.L1
    #     elif 46 < self.ctrl.getPOV(kElevatorButton) < 135:
    #         return ElevatorButton.L2
    #     elif 136 < self.ctrl.getPOV(kElevatorButton) < 225:
    #         return ElevatorButton.L3
    #     elif 226 < self.ctrl.getPOV(kElevatorButton) < 315:
    #         return ElevatorButton.L4
    #     return ElevatorButton.NONE
