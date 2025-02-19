import wpilib
import rev
import math
from math import pi
from rev import SparkMax, AbsoluteEncoder, SparkMaxConfig, EncoderConfig, SparkBase
from wpimath.controller import (
    PIDController,
    ElevatorFeedforward,
    ArmFeedforward,
    ProfiledPIDController,
)
from wpimath.trajectory import TrapezoidProfile
from collections.abc import Callable

from wpiutil import Sendable

from wpilib import DigitalInput

# SHOULDER VALUES
shoulderkP = 3.8
shoulderkI = 0.0
shoulderkD = 0.12

# ELEVATOR VALUES
elevatorkP = 2
elevatorkI = 0
elevatorkD = 0
kDT = 0.02

# ELEVATOR FEEDFORWARD VALUES
kS = 0
kG = 0.04
kV = 0
kA = 0

# WRIST VALUES
wristkP = 0.47
wristkI = 0.04
wristkD = 0

wristkS = 0
wristkG = 0
wristkV = 0.4
wristkA = 0

# INTAKE
intakekS = 0
intakekG = 0
intakekV = 0.2
intakekA = 0


class Shoulder:
    def __init__(self, MotorChannel: int) -> None:
        self.shoulder = SparkMax(MotorChannel, SparkMax.MotorType.kBrushless)

        self.shoulderEncoder = wpilib.DutyCycle(wpilib.DigitalInput(5))

        self.shoulderPid = PIDController(shoulderkP, shoulderkI, shoulderkD)

        self.shoulderPid.setTolerance(0, 0)

        self.shoulder.setInverted(True)

    def set(self, speed: float) -> None:
        self.shoulder.set(speed)

    def getPosition(self) -> float:
        return self.shoulderEncoder.getOutput()

    def calculate(self, measurement: float, setPoint: float) -> float:
        return self.shoulderPid.calculate(measurement, setPoint)

    def setIntegratorRange(self, min: float, max: float) -> None:
        self.shoulderPid.setIntegratorRange(min, max)


class Intake:
    def __init__(self, MotorChannel: int) -> None:
        self.intake = SparkMax(MotorChannel, SparkMax.MotorType.kBrushless)

        self.intakeFeed = ElevatorFeedforward(intakekS, intakekG, intakekV, intakekA)

    def set(self, speed: float) -> None:
        self.intake.set(speed)


class Elevator:
    def __init__(self, MotorChannel: int) -> None:
        self.elevator = SparkMax(MotorChannel, SparkMax.MotorType.kBrushless)

        # self.elevatorEncoder = self.elevator.getEncoder

        self.elevatorEncoder = self.elevator.getAlternateEncoder()

        # self.elevatorPid = PIDController(elevatorkP, elevatorkI, elevatorkD)

        self.elevatorPid = ProfiledPIDController(
            elevatorkP,
            elevatorkI,
            elevatorkD,
            TrapezoidProfile.Constraints(6, 15),
            0.02,
        )

        self.elevator.setInverted(True)

        self.elevatorFeedForward = ElevatorFeedforward(kS, kG, kV, kA)

    def set(self, speed: float) -> None:
        self.elevator.set(speed)

    def setPosition(self, position: float):
        self.elevatorEncoder.setPosition(position)

    def getPosition(self) -> float:
        return self.elevatorEncoder.getPosition()

    def calculate(self, measurement: float, goal: float) -> float:
        return self.elevatorPid.calculate(measurement, goal)

    def setSmartDashboard(self, setName: Callable[[Sendable, str, str], None]) -> None:
        setName(self.elevatorPid, "ElevatorSubsystem", "PID")


class Wrist:
    def __init__(
        self,
        MotorChannel: int
        # , ConversionFactor: float - -0.58544921875
    ) -> None:
        self.wrist = SparkMax(MotorChannel, SparkMax.MotorType.kBrushless)

        self.wristEncoder = wpilib.DutyCycle(wpilib.DigitalInput(6))

        self.wrist.setInverted(True)

        self.wristPid = PIDController(wristkP, wristkI, wristkD)

        self.wristFeedForward = ArmFeedforward(wristkS, wristkG, wristkV, wristkA)

    def set(self, speed: float) -> None:
        self.wrist.set(speed)

    # def setPosition(self, position: float):
    #     self.wristEncoder.setPosition(position)

    def getPosition(self) -> float:
        return self.wristEncoder.getOutput()

    # def getVelocity(self) -> float:
    #     return self.wristEncoder.getVelocity()

    def calculate(self, measurement: float, setPoint: float) -> float:
        return self.wristPid.calculate(measurement, setPoint)

    def setIntegratorRange(self, min: float, max: float) -> None:
        self.wristPid.setIntegratorRange(min, max)


# class Climb:
#     def __init__(self, MotorChannel: int) -> None:
#         self.climb = SparkMax(MotorChannel, SparkMax.MotorType.kBrushless)
#         self.climbEncoder = self.climb.getEncoder()
#     def set(self, speed: float) -> None:
#         self.climb.set(speed)
