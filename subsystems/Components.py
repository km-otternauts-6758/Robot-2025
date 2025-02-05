import wpilib
from rev import SparkMax
from wpimath.controller import PIDController, ElevatorFeedforward, ArmFeedforward
from wpimath.trajectory import TrapezoidProfile

shoulderkP = 0.5
shoulderkI = 0.05
shoulderkD = 0.15

elevatorkP = 0.01
elevatorkI = 0.003
elevatorkD = 0.005

kS = 0
kG = 0.7
kV = 0
kA = 0

wristkP = 0.25
wristkI = 0.0
wristkD = 0.02

wristkS = 0
wristkG = 0
wristkV = 0.4
wristkA = 0


class Shoulder:
    def __init__(self, MotorChannel: int) -> None:
        self.shoulder = SparkMax(MotorChannel, SparkMax.MotorType.kBrushless)

        self.shoulderEncoder = self.shoulder.getEncoder()
        # self.dutyCycle = wpilib.DutyCycle(wpilib.DigitalInput(1))

        self.shoulderPid = PIDController(shoulderkP, shoulderkI, shoulderkD)

        self.shoulderPid.setTolerance(2, 0)

    def set(self, speed: float) -> None:
        self.shoulder.set(speed)

    def setPosition(self, position: float):
        self.shoulderEncoder.setPosition(position)

    def getPosition(self) -> float:
        return self.shoulderEncoder.getPosition()

    def calculate(self, measurement: float, setPoint: float) -> float:
        return self.shoulderPid.calculate(measurement, setPoint)

    def setIntegratorRange(self, min: float, max: float) -> None:
        self.shoulderPid.setIntegratorRange(min, max)


class Intake:
    def __init__(self, MotorChannel: int) -> None:
        self.intake = SparkMax(MotorChannel, SparkMax.MotorType.kBrushless)

    def set(self, speed: float) -> None:
        self.intake.set(speed)


class Elevator:
    def __init__(self, MotorChannel: int) -> None:
        self.elevator = SparkMax(MotorChannel, SparkMax.MotorType.kBrushless)

        self.elevatorEncoder = self.elevator.getEncoder()

        self.elevatorPid = PIDController(elevatorkP, elevatorkI, elevatorkD)

        # self.elevatorPid = PIDController(
        #     elevatorkP, elevatorkI, elevatorkD, TrapezoidProfile.Constraints(1, 1)
        # )

        self.elevator.setInverted(True)

        self.elevatorFeedForward = ElevatorFeedforward(kS, kG, kV, kA)

    def set(self, speed: float) -> None:
        self.elevator.set(speed)

    def setPosition(self, position: float):
        self.elevatorEncoder.setPosition(position)

    def getPosition(self) -> float:
        return self.elevatorEncoder.getPosition()

    def calculate(self, measurement: float, setPoint: float) -> float:
        return self.elevatorPid.calculate(measurement, setPoint)

    def setIntegratorRange(self, min: float, max: float) -> None:
        self.elevatorPid.setIntegratorRange(min, max)


class Wrist:
    def __init__(self, MotorChannel: int) -> None:
        self.wrist = SparkMax(MotorChannel, SparkMax.MotorType.kBrushless)

        self.wristEncoder = self.wrist.getEncoder()

        self.wristPid = PIDController(wristkP, wristkI, wristkD)

        self.wristFeedForward = ArmFeedforward(wristkS, wristkG, wristkV, wristkA)

    def set(self, speed: float) -> None:
        self.wrist.set(speed)

    def setPosition(self, position: float):
        self.wristEncoder.setPosition(position)

    def getPosition(self) -> float:
        return self.wristEncoder.getPosition()

    def calculate(self, measurement: float, setPoint: float) -> float:
        return self.wristPid.calculate(measurement, setPoint)

    def setIntegratorRange(self, min: float, max: float) -> None:
        self.wristPid.setIntegratorRange(min, max)
