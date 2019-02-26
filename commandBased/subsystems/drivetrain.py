

import wpilib
from wpilib.command import Subsystem

from commands.joystickDrive import DriveWithJoystick

from wpilib.drive import MecanumDrive


class DriveTrain(Subsystem):
    """
    The DriveTrain subsystem controls the robot's chassis and reads in
    information about it's speed and position.
    """

    def __init__(self, robot):

        self.robot = robot

        # Configure drive motors
        self.frontLeftCIM = wpilib.Spark(0)
        self.frontRightCIM = wpilib.Spark(2)
        self.backLeftCIM = wpilib.Spark(1)
        self.backRightCIM = wpilib.Spark(3)
        self.joystick = wpilib.Joystick(0)

        self.mecanum_train = MecanumDrive(
            self.frontLeftCIM, self.frontRightCIM, self.backLeftCIM, self.backRightCIM
        )
        self.mecanum_train.setSafetyEnabled(True)
        self.mecanum_train.setExpiration(0.1)
        self.mecanum_train.setMaxOutput(1.0)
        
        # TODO: set the encoder lines at the bottom here
        
        super().__init__()

    def initDefaultCommand(self):
        """
        When other commands aren't using the drivetrain, allow tank drive with
        the self.joystickstick.
        """
        self.setDefaultCommand(DriveWithJoystick(self.robot))

    def arcadeDriveself(self):
        
        self.mecanum_train.driveCartesian(
            self.joystick.getX() * (-self.joystick.getThrottle() + 1) / 2,
            -self.joystick.getY() * (-self.joystick.getThrottle() + 1) / 2,
            self.joystick.getZ() * (-self.joystick.getThrottle() + 1) / 2
        )

    def arcadeDriveManual(self):
        
        self.mecanum_train.driveCartesian(
            self.joystick.getX() * (-self.joystick.getThrottle() + 1) / 2,
            -self.joystick.getY() * (-self.joystick.getThrottle() + 1) / 2,
            self.joystick.getZ() * (-self.joystick.getThrottle() + 1) / 2
        )

    def stop(self):
        """Stop the drivetrain from moving."""
        self.arcadeDriveManual(0, 0)

    # def getLeftEncoder(self):
    #     """:returns: The encoder getting the distance and speed of the right side of the drivetrain."""
    #     return self.leftEncoder
    # 
    # def getRightEncoder(self):
    #     """:returns: The encoder getting the distance and speed of the right side of the drivetrain."""
    #     return self.rightEncoder
    # 
    #  TODO: set the following lines back up at the other TODO
    # Configure encoders
    #         # self.rightEncoder = wpilib.Encoder(
    #         #     1, 2, reverseDirection=False, encodingType=wpilib.Encoder.EncodingType.k4X
    #         # )
    #         # self.leftEncoder = wpilib.Encoder(
    #         #     3, 4, reverseDirection=False, encodingType=wpilib.Encoder.EncodingType.k4X
    #         # )
    #         # self.rightEncoder.setPIDSourceType(wpilib.Encoder.PIDSourceType.kDisplacement)
    #         # self.leftEncoder.setPIDSourceType(wpilib.Encoder.PIDSourceType.kDisplacement)
    #         # 
    #         # if robot.isReal():
    #         #     # Converts to feet
    #         #     self.rightEncoder.setDistancePerPulse(0.0785398)
    #         #     self.leftEncoder.setDistancePerPulse(0.0785398)
    #         # else:
    #         #     # Convert to feet 4in diameter wheels with 360 tick simulated encoders.
    #         #     self.rightEncoder.setDistancePerPulse((6 * math.pi) / (360 * 12))
    #         #     self.leftEncoder.setDistancePerPulse((6 * math.pi) / (360 * 12))
    #         # 
    #         # wpilib.LiveWindow.addSensor("DriveTrain", "Right Encoder", self.rightEncoder)
    #         # wpilib.LiveWindow.addSensor("DriveTrain", "Left Encoder", self.leftEncoder)
