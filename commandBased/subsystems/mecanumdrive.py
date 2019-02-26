import wpilib
from wpilib.command.subsystem import Subsystem
from wpilib.drive import MecanumDrive


class MecanumMove (Subsystem):

    def __init__(self):

        super().__init__("MecanumMove")

        self.FrontLeft = wpilib.Spark(0)
        self.FrontRight = wpilib.Spark(2)
        self.RearLeft = wpilib.Spark(1)
        self.RearRight = wpilib.Spark(3)

        self.drive = MecanumDrive(self.FrontLeft,
                                  self.RearLeft,
                                  self.FrontRight,
                                  self.RearRight
                                  )

    def setSpeed(self, speed):
        self.FrontLeft.set(speed)
        self.FrontRight.set(speed)
        self.RearLeft.set(speed)
        self.RearRight.set(speed)
