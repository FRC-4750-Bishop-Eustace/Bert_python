import math
import time
import sys

import wpilib
import wpilib.drive
import ctre 

# HARDWARE DEF

#from wpilib.drive import DifferentialDrive


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""

        # object that handles basic drive operations
        # Check for the right motor definition (Talon or other)
        
        '''
        self.frontLeftMotor = wpilib.Talon(0)
        self.rearLeftMotor = wpilib.Talon(1)
        self.frontRightMotor = wpilib.Talon(15)
        self.rearRightMotor = wpilib.Talon(14)
        '''
        # Phoenix Tuner
        # PID 1: Right Back; PID 2: Right Front; PID 14: Left Front; PID 15: Left Back; PID 0: PDP
        ''' 
        self.frontLeftMotor = wpilib.Talon(15)
        self.rearLeftMotor = wpilib.Talon(14)
        self.frontRightMotor = wpilib.Talon(2)
        self.rearRightMotor = wpilib.Talon(1)
        '''
        
        self.frontLeftMotor = ctre.WPI_TalonSRX(0)
        #self.frontLeftMotor.set(0.3)
        self.rearLeftMotor = ctre.WPI_TalonSRX(1)
        #self.rearLeftMotor.set(0.3)
        self.frontRightMotor = ctre.WPI_TalonSRX(15)
        #self.frontRightMotor.set(0.3)
        self.rearRightMotor = ctre.WPI_TalonSRX(14)
        #self.rearRightMotor.set(0.3)
        
        '''
        self.frontLeftMotor = ctre.WPI_TalonSRX(0)
        #self.frontLeftMotor.set(0.3)
        self.rearLeftMotor = ctre.WPI_TalonSRX(1)
        #self.rearLeftMotor.set(0.3)
        self.frontRightMotor = ctre.WPI_TalonSRX(15)
        #self.frontRightMotor.set(0.3)
        self.rearRightMotor = ctre.WPI_TalonSRX(14)
        #self.rearRightMotor.set(0.3)
        '''
        self.m_left = wpilib.MotorControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.m_right = wpilib.MotorControllerGroup(self.frontRightMotor, self.rearRightMotor)
        
        self.m_left.setInverted(True)

        #self.myRobot = DifferentialDrive(self.left, self.right)
        #self.myRobot.setExpiration(0.1)
        
        #self.myRobot = wpilib.drive.DifferentialDrive(self.left, self.right)
        self.driveTrain = wpilib.drive.DifferentialDrive(self.m_left, self.m_right)
        self.driveTrain.setExpiration(0.1)
        #self.myRobot.setExpiration(0.1)

        # joysticks 1 & 2 on the driver station
        self.joystick = wpilib.Joystick(0)
        #self.rightStick = wpilib.Joystick(1)
        self.timer = wpilib.Timer()

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        #self.myRobot.setSafetyEnabled(True)
        self.driveTrain.setSafetyEnabled(True)
        #self.driveTrain.setSafetyEnabled(False)
        self.driveTrain.setExpiration(0.1)
        self.driveTrain.feed()

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        #self.myRobot.tankDrive(self.leftStick.getY() * -1, self.rightStick.getY() * -1)
        i = 0
        if i==0:
            print("In TeleopMode")
            i=i+1
        #self.driveTrain.arcadeDrive(-0.5, 0)
        #print("Driving")
        #time.sleep(1)
        #print("Sleep Complete")
        #self.driveTrain.arcadeDrive(0,0)
        #self.myRobot.arcadeDrive(
        #    self.stick.getRawAxis(0), self.stick.getRawAxis(1), True
        #)
        print(self.joystick.getY())
        print(self.joystick.getX())
        self.driveTrain.arcadeDrive(-self.joystick.getY(), self.joystick.getX())
    ''' TEST class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        self.frontLeft = wpilib.Spark(0)
        self.rearLeft = wpilib.Spark(1)
        self.left = wpilib.MotorControllerGroup(self.frontLeft, self.rearLeft)

        self.frontRight = wpilib.Spark(15)
        self.rearRight = wpilib.Spark(14)
        self.right = wpilib.MotorControllerGroup(self.frontRight, self.rearRight)

        self.drive = wpilib.drive.DifferentialDrive(self.left, self.right)
    '''
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        #self.driveTrain.setSafetyEnabled(True)
        self.driveTrain.setSafetyEnabled(False)
        self.driveTrain.setExpiration(0.1)
        self.driveTrain.feed()
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        '''
        print("Autonomous Mode")
        self.driveTrain.arcadeDrive(-0.5, 0)
        #print("Driving")
        time.sleep(1)
        #print("Sleep Complete")
        self.driveTrain.arcadeDrive(0,0)
        # Drive for two seconds
        '''
        i = 0
        if i==0:
            print("In Autonomous Mode")
            i=i+1
        #self.driveTrain.arcadeDrive(-0.5, 0)
        #time.sleep(0.5)
        #self.driveTrain.arcadeDrive(0, 0)
        
        if self.timer.get() < 2.0:
            self.driveTrain.arcadeDrive(-0.5, 0)  # Drive forwards at half speed
        else:
            self.driveTrain.arcadeDrive(0, 0)  # Stop robot
        
'''
    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.drive.arcadeDrive(self.stick.getY(), self.stick.getX())
'''

# MAIN LOOP

if __name__ == '__main__':
    print("Working")
    wpilib.run(MyRobot)
   