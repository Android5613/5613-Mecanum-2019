#!/usr/bin/env python3
'''
    This is a demo program showing how to use Mecanum control with the
    MecanumDrive class.
'''

import wpilib
from wpilib.drive import MecanumDrive

class MyRobot(wpilib.SampleRobot):
    # Channels on the roboRIO that the motor controllers are plugged in to
    frontLeftChannel = 0
    rearLeftChannel = 2
    frontRightChannel = 1
    rearRightChannel = 3

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0

    def robotInit(self):
        wpilib.CameraServer.launch()
        '''Robot initialization function'''
        self.frontLeftMotor = wpilib.Spark(self.frontLeftChannel)
        self.rearLeftMotor = wpilib.Spark(self.rearLeftChannel)
        self.frontRightMotor = wpilib.Spark(self.frontRightChannel)
        self.rearRightMotor = wpilib.Spark(self.rearRightChannel)

        # invert the left side motors
        self.frontRightMotor.setInverted(True)

        # you may need to change or remove this to match your robot
        self.rearRightMotor.setInverted(True)

        self.drive = MecanumDrive(self.frontLeftMotor,
                                  self.rearLeftMotor,
                                  self.frontRightMotor,
                                  self.rearRightMotor)

        self.drive.setExpiration(0.1)

        self.stick = wpilib.Joystick(self.joystickChannel)

    def operatorControl(self):
        '''Runs the motors with Mecanum drive.'''

        self.drive.setSafetyEnabled(True)
        while self.isOperatorControl() and self.isEnabled():

            self.drive.driveCartesian(
                    self.stick.getY() * (-self.stick.getThrottle()+1)/2,
                    -self.stick.getX() * (-self.stick.getThrottle()+1)/2,
                    -self.stick.getZ() * (-self.stick.getThrottle()+1)/2
            )

            wpilib.Timer.delay(0.005)  # wait 5ms to avoid hogging CPU cycles


if __name__ == '__main__':
    wpilib.run(MyRobot)
