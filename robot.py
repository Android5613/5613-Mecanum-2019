#!/usr/bin/env python3
import ctre
import navx
import wpilib
import wpilib.buttons
from wpilib.drive import MecanumDrive
from robotpy_ext import autonomous


class MyRobot(wpilib.IterativeRobot):
    # Channels on the roboRIO that the motor controllers are plugged in to
    frontLeftChannel = 0
    rearLeftChannel = 2
    frontRightChannel = 1
    rearRightChannel = 3

    # CAN bus identities of the CAN bus motor controllers.
    CargoPivotChannel1 = 0
    CargoPivotChannel2 = 1
    CargoIntakeChannel1 = 3
    CargoIntakeChannel2 = 2

    # Channel of the winch used to lift the lotus module.
    lotusWinchChannel = 4

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0
    functStickChannel = 1

    def robotInit(self):
        # launches our cameras
        wpilib.CameraServer.launch()

        self.automodes = autonomous.AutonomousModeSelector("autonomous")

        # sets up the smartdashboard and timer for the navx gyro system.
        self.sd = wpilib.SmartDashboard
        self.timer = wpilib.Timer()

        # sets up the navx itself. There are two different connection methods, hence why I left the other commented out.
        self.navx = navx.AHRS.create_spi()
        # self.navx = navx.AHRS.create_i2c()

        self.analog = wpilib.AnalogInput(navx.getNavxAnalogInChannel(0))

        # sets up the drivetrain motors on our robot.
        frontLeftMotor = wpilib.Spark(self.frontLeftChannel)
        rearLeftMotor = wpilib.Spark(self.rearLeftChannel)
        frontRightMotor = wpilib.Spark(self.frontRightChannel)
        rearRightMotor = wpilib.Spark(self.rearRightChannel)

        # sets up the other motors on our robot.
        self.lotusWinch = wpilib.Spark(self.lotusWinchChannel)

        self.CargoPivot1 = ctre.victorspx.VictorSPX(self.CargoPivotChannel1)
        self.CargoPivot2 = ctre.victorspx.VictorSPX(self.CargoPivotChannel2)
        self.CargoIntake1 = ctre.victorspx.VictorSPX(self.CargoIntakeChannel1)
        self.CargoIntake2 = ctre.victorspx.VictorSPX(self.CargoIntakeChannel2)

        # sets up the joysticks to control the robot.
        self.driveStick = wpilib.Joystick(self.joystickChannel)
        self.functStick = wpilib.Joystick(self.functStickChannel)

        # sets up the solenoids to run the lotus.
        self.LotusOut = wpilib.Solenoid(0, 0)
        self.LotusIn = wpilib.Solenoid(0, 1)

        # sets our motors into the drive function.
        self.drive = MecanumDrive(frontLeftMotor,
                                  rearLeftMotor,
                                  frontRightMotor,
                                  rearRightMotor)

        # turns off motors after 0.1 seconds
        self.drive.setExpiration(0.1)

    # autonomous mode. INCOMPLETE, DO NOT USE!!!
    # todo: complete autonomous mode
    # def autonomous(self):
    #
    #     self.autoTimer = wpilib.Timer.getMsClock()
    #
    #     if self.autoTimer > 2:
    #         self.lotusWinch.set(-0.5)
    #     elif self.autoTimer > 3:
    #         self.lotusWinch.set(1)
    #     else:
    #         self.lotusWinch.set(0)
    def autonomousPeriodic(self):
        self.automodes.run()



    def operatorControl(self):

        # non functional takeover functional, not to important, so i will finish it later.
        # todo: finish the driver takeover
        # if self.functStick.getRawButton(3):
        #     self.driveStick = 0

        # freezes the driver if they are out of control. this is useful if they are doing something stupid.
        if self.functStick.getRawButton(4):
            self.driveStick = 0
        else:
            self.driveStick = wpilib.Joystick(self.joystickChannel)

        # enables safety.
        self.drive.setSafetyEnabled(True)
        while self.isOperatorControl() and self.isEnabled():

            # alternate drive functions for the driver
            # sideways only
            if self.driveStick.getRawButton(1):
                self.drive.driveCartesian(
                    self.driveStick.getX() * (-self.driveStick.getThrottle(
                    ) + 1) / 2,
                    0,
                    0
                )
            # sideways only
            elif self.driveStick.getRawButton(2):
                self.drive.driveCartesian(
                    0,
                    self.driveStick.getY() * (-self.driveStick.getThrottle(
                    ) + 1) / 2,
                    0
                )
            # turning only
            elif self.driveStick.getRawButton(7):
                self.drive.driveCartesian(
                    0,
                    0,
                    self.driveStick.getZ() * (-self.driveStick.getThrottle(
                    ) + 1) / 2
                )
            # normal drive function if no button is being pressed
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

            # these lines of code are not needed because there is an alternate control method already implemented.
            # designed to run the cargo handler with buttons.
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
            # up and down functions for the lotus elevator.
            if self.functStick.getRawButton(2):
                self.lotusWinch.set(-1.0)

            elif self.functStick.getRawButton(7):
                self.lotusWinch.set(1.0)

            else:
                self.lotusWinch.set(0)

            # sets the cargo intake motors to be a speed controlled by the throttle on a joystick
            if self.functStick.getRawButton(5):
                self.CargoIntake1.set(ctre.ControlMode.PercentOutput,
                                      1 * (
                                              -self.functStick.getThrottle() + 1) / 2)
                self.CargoIntake2.set(ctre.ControlMode.PercentOutput,
                                      -1 * (
                                              -self.functStick.getThrottle() + 1) / 2)
            # runs the cargo intake the other way.
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

            # moves our cargo handler based on the y axis of the controller.
            self.CargoPivot1.set(ctre.ControlMode.PercentOutput,
                                 self.functStick.getY())

            self.CargoPivot2.set(ctre.ControlMode.PercentOutput,
                                 -self.functStick.getY())

            # opens and closes the lotus using the trigger.
            # start in the open position and closes when you press the trigger.
            self.LotusIn.set(not self.functStick.getTrigger())
            self.LotusOut.set(self.functStick.getTrigger())

            # puts values into the smartdashboard.
            # todo: get smardashboard
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
