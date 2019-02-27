#!/usr/bin/env python3
# added detail for test
# second added detail for test
import ctre
import navx
import wpilib
import wpilib.buttons
from wpilib.drive import MecanumDrive


class MyRobot(wpilib.SampleRobot):
    # Channels on the roboRIO that the motor controllers are plugged in to
    frontLeftChannel = 0
    rearLeftChannel = 2
    frontRightChannel = 1
    rearRightChannel = 3

    CargoPivotChannel1 = 0
    CargoPivotChannel2 = 1
    CargoIntakeChannel1 = 2
    CargoIntakeChannel2 = 3

    lotusWinchChannel = 4

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0
    functStickChannel = 1

    def robotInit(self):
        wpilib.CameraServer.launch()

        self.sd = wpilib.SmartDashboard
        self.timer = wpilib.Timer()

        self.navx = navx.AHRS.create_spi()
        # self.navx = navx.AHRS.create_i2c()

        self.analog = wpilib.AnalogInput(navx.getNavxAnalogInChannel(0))

        frontLeftMotor = wpilib.Spark(self.frontLeftChannel)
        rearLeftMotor = wpilib.Spark(self.rearLeftChannel)
        frontRightMotor = wpilib.Spark(self.frontRightChannel)
        rearRightMotor = wpilib.Spark(self.rearRightChannel)

        self.lotusWinch = wpilib.Spark(self.lotusWinchChannel)

        self.CargoPivot1 = ctre.victorspx.VictorSPX(self.CargoPivotChannel1)
        self.CargoPivot2 = ctre.victorspx.VictorSPX(self.CargoPivotChannel2)
        self.CargoIntake1 = ctre.victorspx.VictorSPX(self.CargoIntakeChannel1)
        self.CargoIntake2 = ctre.victorspx.VictorSPX(self.CargoIntakeChannel2)

        self.driveStick = wpilib.Joystick(self.joystickChannel)
        self.functStick = wpilib.Joystick(self.functStickChannel)

        self.LotusOut = wpilib.Solenoid(0, 0)
        self.LotusIn = wpilib.Solenoid(0, 1)

        self.drive = MecanumDrive(frontLeftMotor,
                                  rearLeftMotor,
                                  frontRightMotor,
                                  rearRightMotor)

        self.drive.setExpiration(0.1)

    def autonomous(self):

        self.autoTimer = self.timer.getMsClock()

        if self.autoTimer > 2:
            self.lotusWinch.set(-0.5)
        elif self.autoTimer > 3:
            self.lotusWinch.set(1)
        else:
            self.lotusWinch.set(0)

    def operatorControl(self):
        # Runs the motors with Mecanum drive.
        # if self.functStick.getRawButton(3):
        #     self.driveStick = 0
        if self.functStick.getRawButton(4):
            self.driveStick = 0

        else:
            self.driveStick = wpilib.Joystick(self.joystickChannel)
            self.functStick = wpilib.Joystick(self.functStickChannel)

        self.drive.setSafetyEnabled(True)
        while self.isOperatorControl() and self.isEnabled():

            if self.driveStick.getRawButton(1):
                self.drive.driveCartesian(
                    self.driveStick.getX() * (-self.driveStick.getThrottle(
                    ) + 1) / 2,
                    0,
                    0
                )
            elif self.driveStick.getRawButton(2):
                self.drive.driveCartesian(
                    0,
                    self.driveStick.getY() * (-self.driveStick.getThrottle(
                    ) + 1) / 2,
                    0
                )
            elif self.driveStick.getRawButton(7):
                self.drive.driveCartesian(
                    0,
                    0,
                    self.driveStick.getZ() * (-self.driveStick.getThrottle(
                    ) + 1) / 2
                )

            else:
                self.drive.driveCartesian(
                    self.driveStick.getX() * (
                            -self.driveStick.getThrottle() + 1)
                    / 2,
                    -self.driveStick.getY() * (
                            -self.driveStick.getThrottle() + 1)
                    / 2,
                    self.driveStick.getZ() * (
                            -self.driveStick.getThrottle() + 1) / 2
                )

            # if self.stick.getRawButton(6):
            #     self.CargoPivot1.set(ctre.ControlMode.PercentOutput, 1)
            #     self.CargoPivot2.set(ctre.ControlMode.PercentOutput, -1)
            # elif self.stick.getRawButton(2):
            #     self.CargoPivot1.set(ctre.ControlMode.PercentOutput, -1)
            #     self.CargoPivot2.set(ctre.ControlMode.PercentOutput, 1)
            # else:
            #     self.CargoPivot1.set(ctre.ControlMode.PercentOutput, 1)
            #     self.CargoPivot1.set(ctre.ControlMode.PercentOutput, 1)
            #
            if self.functStick.getRawButton(2):
                self.lotusWinch.set(-1.0)

            elif self.functStick.getRawButton(7):
                self.lotusWinch.set(1.0)

            else:
                self.lotusWinch.set(0)

            if self.functStick.getRawButton(5):
                self.CargoIntake1.set(ctre.ControlMode.PercentOutput,
                                      1 * (
                                              self.functStick.getThrottle() + 1) / 2)
                self.CargoIntake2.set(ctre.ControlMode.PercentOutput,
                                      -1 * (
                                              self.functStick.getThrottle() + 1) / 2)
            elif self.functStick.getRawButton(6):
                self.CargoIntake1.set(ctre.ControlMode.PercentOutput,
                                      -1 * (
                                              -self.functStick.getThrottle() + 1) / 2)
                self.CargoIntake2.set(ctre.ControlMode.PercentOutput,
                                      1 * (
                                              -self.functStick.getThrottle() + 1) / 2)
            else:
                self.CargoIntake1.set(ctre.ControlMode.PercentOutput, 0)
                self.CargoIntake2.set(ctre.ControlMode.PercentOutput, 0)

            self.CargoPivot1.set(ctre.ControlMode.PercentOutput,
                                 self.functStick.getY())

            self.CargoPivot2.set(ctre.ControlMode.PercentOutput,
                                 -self.functStick.getY())

            self.LotusIn.set(not self.functStick.getTrigger())
            self.LotusOut.set(self.functStick.getTrigger())

            self.sd.putBoolean("IsCalibrating", self.navx.isCalibrating())
            self.sd.putBoolean("IsConnected", self.navx.isConnected())
            self.sd.putNumber("Angle", self.navx.getAngle())
            self.sd.putNumber("Pitch", self.navx.getPitch())
            self.sd.putNumber("Yaw", self.navx.getYaw())
            self.sd.putNumber("Roll", self.navx.getRoll())
            self.sd.putNumber("Analog", self.analog.getVoltage())
            self.sd.putNumber("Timestamp", self.navx.getLastSensorTimestamp())

            wpilib.Timer.delay(0.005)  # wait 5ms to avoid hogging CPU cycles


if __name__ == '__main__':
    wpilib.run(MyRobot)
