import math
import time
import sys
import logging
import threading

import wpilib
import wpilib.drive
import ctre 
import navx
#import VL53L1X

import ntcore 
import logging 
import cscore

import math


#from networktables import NetworkTables

# HARDWARE DEF

#from wpilib.drive import DifferentialDrive

#logging.basicConfig(level=logging.DEBUG)

#GLOBAL DEF
minTX = -1
maxTX = 1
autoGo = 4
autoStop = 3.9
autoAimLeft = 4
autoAimRight = -4

kp = -0.1
min_command = 0.05
steeringAdjust = 0.0

FrontLeftMotorPort = 1
RearLeftMotorPort = 2
FrontRightMotorPort = 12
RearRightMotorPort = 13
ArmExtensionMotorPort = 2
AngleMotorPort = 12

ArmStopVar = 500
ArmLengthVar = 660
ArmLengthWarning = 600
ArmAngleAutoVar = -1500
#for "event.wait" to work
event = threading.Event()

#Distance equation
def targetDistance(x):
    a = 4.509
    b = -0.5132
    distance = 0
    if x > 0:
        distance = a * x ** b
    else:
        pass
    return distance

def targetAlignment(x):
    isTargetAligned = ''
    if (minTX <= x <= maxTX):
        #print("Aligned")
        isTargetAligned = 'Aligned'
    else:
        #print("unaligned")
        isTargetAligned = 'Unaligned'
    return isTargetAligned

def ArmAngle(x):
    y = x - 118.3/-103.5
    return y

def armExtension(x):
    runningTotal = 0
    avgFactor = 75
    for y in range(avgFactor):
        distance = 28250 / (x-229.5)
        runningTotal = runningTotal + distance
    else:
        distance = (runningTotal + distance)
    return distance

def solenoidclaw(x):
    isSolenoidClawOpen = 'undefined'
    if (x==1):
        #print ("Claw Open")
        isSolenoidClawOpen = 'Open'
    else:
        #print ("Claw Close")
        isSolenoidClawOpen = 'Close'
    print('returningsolenoidclaw')
    return isSolenoidClawOpen

def pressure_status(x):
    y = 0
    if (x < 388):
        y = 0
    elif 388 <= x < 392:
        y = 1
    if x >= 392:
        y = 2
    return y

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        print("AtRobotInitBeginning")
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
        #
        self.frontLeftMotor = ctre.WPI_TalonSRX(15)
        #self.frontLeftMotor.set(0.3)
        self.rearLeftMotor = ctre.WPI_TalonSRX(1)
        #self.rearLeftMotor.set(0.3)
        self.frontRightMotor = ctre.WPI_TalonSRX(14)
        #self.frontRightMotor.set(0.3)
        self.rearRightMotor = ctre.WPI_TalonSRX(5)
        #self.rearRightMotor.set(0.3)
        self.armUpDown = ctre.WPI_TalonSRX(2)
        self.armExtend = ctre.WPI_TalonSRX(0)
        '''
        self.frontLeftMotor = ctre.WPI_TalonSRX(0)
        #self.frontLeftMotor.set(0.3)
        self.rearLeftMotor = ctre.WPI_TalonSRX(1)
        #self.rearLeftMotor/.set(0.3)
        self.frontRightMotor = ctre.WPI_TalonSRX(15)
        #self.frontRightMotor.set(0.3)
        self.rearRightMotor = ctre.WPI_TalonSRX(14)
        self.rearRightMotor.set(0.3)
        '''
        self.m_left = wpilib.MotorControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.m_right = wpilib.MotorControllerGroup(self.frontRightMotor, self.rearRightMotor)
        
        self.m_left.setInverted(True)
        #self.m_left.setVoltage(5)
        #self.m_right.setVoltage(7)

        #self.myRobot = DifferentialDrive(self.left, self.right)
        #self.myRobot.setExpiration(0.1)
        
        #self.myRobot = wpilib.drive.DifferentialDrive(self.left, self.right)
        self.driveTrain = wpilib.drive.DifferentialDrive(self.m_left, self.m_right)
        #self.driveTrain.WheelSpeeds()
        self.driveTrain.setExpiration(0.1)
        #self.myRobot.setExpiration(0.1)

        # joysticks 1 & 2 on the driver station
        self.joystick = wpilib.Joystick(0)
        #self.controller = wpilib.Joystick(1)
        self.controller = wpilib.XboxController(2)
        self.timer = wpilib.Timer()

        self.claw = 0

         ## SOLENOID TESTING
        self.doubleSolenoid = wpilib.DoubleSolenoid(3,wpilib.PneumaticsModuleType.CTREPCM, 4,5)
        kForward = 5
        kOff = 4
        kReverse = 2
        
        ## DEFINE COMPRESSOR
        self.compressor = wpilib.Compressor(3,wpilib.PneumaticsModuleType.CTREPCM)
        
        ## ENCODER DEFINITION
        self.UpDownEncoder = self.armUpDown.getSelectedSensorPosition(0)
        self.ExtendEncoder = self.armExtend.getSelectedSensorPosition(0)
        
        # Encoder Testing
        self.kTimeoutMs = 0
        self.kPIDLoopIdx = 0
        self.kSlotIdx = 0
        #self.rearRightMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.PulseWidthEncodedPosition, 0, 0)
        #self.rearRightMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)
        #self.rearRightMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        #self.rearRightMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0)
        
        # Drive Encoder 1
        self.rearLeftMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0,)
        self.rearLeftMotor.setSensorPhase(True)
        self.rearLeftMotor.setSelectedSensorPosition(100,0,0)

        # Drive Encoder 2
        self.frontRightMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0,)
        self.frontRightMotor.setSensorPhase(True)
        self.frontRightMotor.setSelectedSensorPosition(100,0,0)
        # Set relevant frame periods to be at least as fast as periodic rate
        ##self.rearRightMotor.setStatusFramePeriod(ctre.WPI_TalonSRX.StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0)
        ##self.rearRightMotor.setStatusFramePeriod(ctre.WPI_TalonSRX.StatusFrameEnhanced.Status_10_MotionMagic, 10, self.kTimeoutMs)
        self.armUpDown.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0,)
        self.armUpDown.setSensorPhase(True)
        self.armUpDown.setSelectedSensorPosition(100,0,0)

        #encoder for extention
        self.armExtend.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0,)
        self.armExtend.setSensorPhase(True)
        self.armExtend.setSelectedSensorPosition(100,0,0)


        # set the peak and nominal outputs
        self.frontLeftMotor.configNominalOutputForward(0, self.kTimeoutMs)
        self.frontLeftMotor.configNominalOutputReverse(0, self.kTimeoutMs)
        self.frontLeftMotor.configPeakOutputForward(1, self.kTimeoutMs)
        self.frontLeftMotor.configPeakOutputReverse(-1, self.kTimeoutMs)

        # set closed loop gains in slot0 - see documentation */
        self.frontLeftMotor.selectProfileSlot(self.kSlotIdx, self.kPIDLoopIdx)
        self.frontLeftMotor.config_kF(0, 0.2, self.kTimeoutMs)
        self.frontLeftMotor.config_kP(0, 0.2, self.kTimeoutMs)
        self.frontLeftMotor.config_kI(0, 0, self.kTimeoutMs)
        self.frontLeftMotor.config_kD(0, 0, self.kTimeoutMs)
        # set acceleration and vcruise velocity - see documentation
        self.frontLeftMotor.configMotionCruiseVelocity(15000, self.kTimeoutMs)
        self.frontLeftMotor.configMotionAcceleration(6000, self.kTimeoutMs)
        # zero the sensor
        self.frontLeftMotor.setSelectedSensorPosition(0, self.kPIDLoopIdx, self.kTimeoutMs)

        
        ## NETWORK TABLES
        self.inst = ntcore.NetworkTableInstance.getDefault()
        #self.inst.startServer()
        #self.inst = ntcore.NetworkTableInstance.create()
        self.sd = self.inst.getTable("SmartDashboard")
        self.lmtable = self.inst.getTable("limelight")
        self.station = self.inst.getTable("FMSInfo")
        #self.lltable = ntcore.NetworkTableInstance
        
        ## DEFINE NAVX
        #self.navx = navx.AHRS.create_i2c(wpilib.I2C.Port.kMXP, update_rate_hz=80)
        self.navxer = navx.AHRS.create_i2c()
        print("NAVX firmware = ", navx.AHRS.getFirmwareVersion)
        #print("I2C device address is {}".format(wpilib.I2C.getDeviceAddress()))
        #print("I2C address only {}".format(wpilib.I2C.addressOnly()))
        
        ## DEFINE LIMELIGHT
        # Mr. Carlin's genius helped find this "da" moment, had to add the call to teleop
        
        #"""Executed at the start of teleop mode"""
        #self.myRobot.setSafetyEnabled(True)
        self.driveTrain.setSafetyEnabled(True)
        self.driveTrain.setSafetyEnabled(False)
        self.driveTrain.setExpiration(0.1)
        self.driveTrain.feed()
        # Launch Camera
        wpilib.CameraServer.launch()

        #ARM EXTENTION
        #self.extension = wpilib.AnalogInput(0)

        #PSI SENSOR
        self.psi = wpilib.AnalogInput(2)

        #EXTENTION SENSOR
        self.armStop = wpilib.AnalogInput(1)

        #navx micro for angles
        self.angler = navx.AHRS.create_i2c(wpilib.I2C.Port.kOnboard)

        self.armExtensionMotor = ctre.WPI_TalonSRX(ArmExtensionMotorPort)
        self.angleMotor = ctre.WPI_TalonSRX(AngleMotorPort)

        #self.armExtensionMotor = ctre.WPI_TalonSRX(2)
        #self.angleMotor = ctre.WPI_TalonSRX(12)

        self.armDirection = 1

        self.clawStatus = 0

        print("AtRobotInitEnd")

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        #self.myRobot.tankDrive(self.leftStick.getY() * -1, self.rightStick.getY() * -1)
        i = 0
        #if i==0:
        #    print("In TeleopMode")
        #    i=i+1
        #self.driveTrain.arcadeDrive(-0.5, 0)
        #print("Driving")
        #time.sleep(1)
        #print("Sleep Complete")
        #self.driveTrain.arcadeDrive(0,0)
        #self.myRobot.arcadeDrive(
        #    self.stick.getRawAxis(0), self.stick.getRawAxis(1), True
        #)
        #print(self.joystick.getY())
        #print(self.joystick.getX())
        #print(self.joystick.getRawButtonPressed(1))
        
        #LIMELIGHT Variables
        self.tx = self.lmtable.getNumber('tx', None)
        self.ty = self.lmtable.getNumber('ty', None)
        self.ta = self.lmtable.getNumber('ta', None)
        self.ts = self.lmtable.getNumber('ts', None)
        self.tid = self.lmtable.getNumber('tid', None)
        #self.cl = self.lmtable.getNumber('cl', None)

        self.limelightLensHeightInches = 14

        #target distance to dashborard
        self.distance = targetDistance(self.ta)
        self.sd.putNumber('tDistance', self.distance)

        #target alignment pushing to dashboard
        self.aim = targetAlignment(self.tx)
        self.sd.putString('tAim', self.aim)

        #arm extension to dashboard
        self.armStopValue = self.armStop.getValue()
        self.sd.putNumber('arm length', self.armStopValue)
        
        #temp encoder values to dashboard
        self.sd.putNumber('arm up down', self.armUpDown.getSelectedSensorPosition(0))
        self.sd.putNumber('rear left', self.rearLeftMotor.getSelectedSensorPosition(0))
        self.sd.putNumber('front right', self.frontRightMotor.getSelectedSensorPosition(0))
        #self.sd.putNumber('extend arm', self.armExtend.getSelectedSensorPosition(0))

        #psi status to dashboard
        self.psi_status = pressure_status(self.psi.getValue())
        self.sd.putNumber('pressureStatus', self.psi_status)

        #navx angle to dashboard
        self.armangle = self.angler.getAngle()
        self.sd.putNumber('armAngle', self.armangle)

        #claw open or close dashboard
        '''
        self.clawDash = 'close'
        if (self.clawStatus == 0):
            self.clawDash = 'close'
        if (self.clawStatus == 1):
            self.clawDash == 'open'
        self.sd.putString('Claw', self.clawDash)
        '''
        #arm extention flag
        self.armLength = 0
        if (self.armStopValue > ArmLengthVar):
            self.armLength = 1
        if (self.armStopValue < ArmLengthVar):
            self.armLength = 0
        
        #arm extention flag for not dragging on the ground
        if (self.armStopValue > ArmLengthWarning):
            self.sd.putString('ArmStatus', '')
        if (self.armStopValue < ArmLengthWarning):
            self.sd.putString('ArmStatus', 'WARN')

            
        #getting arm extention values
        self.armStopValue0 = self.armStop.getValue()
        self.armStopValue1 = self.armStop.getValue()

        #get claw status
        self.clawStatus = 0

        if self.joystick.getRawButtonPressed(1):
            print("Button 1 Pressed")
            #self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        if self.joystick.getRawButtonPressed(2):
            print("Button 2 Pressed")
            #self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kOff)
            self.driveTrain.curvatureDrive(self.joystick.getY(), self.joystick.getRawAxis(3) * 1/2, True)
        if self.joystick.getRawButtonPressed(3):
            print("Button 3 Pressed")
            #self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
            self.driveTrain.curvatureDrive(self.joystick.getY(), self.joystick.getRawAxis(3) * 1/2, True)
        if self.joystick.getRawButtonPressed(4):
            print("Button 4 Pressed")
            #print("solenoid value = ",self.doubleSolenoid.get())
            # NAVX uses network tables as well
            self.getY = self.navxer.getRawAccelY()
            self.angle = self.navxer.getAngle()
            self.getX = self.navxer.getRawAccelX()
            print("navx X = ", self.getX)
            print("navx Y = ", self.getY)  
            print("Anle = ", self.angle)  
        if self.joystick.getRawButtonPressed(5):
            print("Button 5 Pressed")
            '''
            print("CTRE PulseWidthEncoded Position := ", self.encoder.PulseWidthEncodedPosition.value)
            print("QuadEncoder = ", self.encoder.QuadEncoder.value)
            sensorreadout = self.rearRightMotor.getSensorCollection()
            print("Sensor Collection Analog In Raw = ", sensorreadout.getAnalogInRaw())
            print("Sensor Collection Pulse Width Pos = ", sensorreadout.getPulseWidthPosition())
            print("Sensor Collection Pulse Width Rise to Fall = ", sensorreadout.getPulseWidthRiseToFallUs())
            print("Sensor Collection QuadIdx = ", sensorreadout.getPinStateQuadIdx())
            print("Sensor Collection Pulse Width Velocity = ", sensorreadout.getPulseWidthVelocity())
            print("Sensor Collection Quad Position = ", sensorreadout.getQuadraturePosition())
            print("Sensor Collection Quad Velocity = ", sensorreadout.getQuadratureVelocity)
            '''
            # Encoder Testing
            #print("Sensor Position", self.frontLeftMotor.getSelectedSensorPosition(0))
            print("arm", self.armUpDown.getSelectedSensorPosition(0))
            print("left", self.rearLeftMotor.getSelectedSensorPosition(0))
            print("right", self.frontRightMotor.getSelectedSensorPosition(0))
        if self.joystick.getRawButtonPressed(6):
            print("Button 6 Pressed")
            self.compressor.disable()
        if self.joystick.getRawButtonPressed(7):
            #Limelight
            print("Button 7 Pressed")
            self.PadY = self.controller.getRawAxis(1)
            print("joystick =", self.PadY)
            
        
        self.driveTrain.curvatureDrive(self.joystick.getY(), self.joystick.getRawAxis(3) * 1/2, True)

        #CONTROLLER JOYSTICK
        self.PadX = self.controller.getRawAxis(3)
        self.PadY = self.controller.getRawAxis(1)
        #controls up down
        if (self.PadY == 1):
            self.armUpDown.set(1.2)
        elif (self.PadY == -1):
            self.armUpDown.set(-1.2)
        else:
            self.armUpDown.set(0.0)
        
        #controls extend (negive is joy stick moving left: positive is joystick moving right)
        if (self.armLength == 0):
            if (self.PadX == -1):
                self.armExtend.set(0.9)
            elif (self.PadX == 1):
                self.armExtend.set(-0.9)
            else:
                self.armExtend.set(0.0)
        #stop the arm extension if its fully closed
        if (self.armLength == 1):
            if (self.PadX == -1):
                self.armExtend.set(0.0)
            elif (self.PadX == 1):
                self.armExtend.set(0.9)
            else:
                self.armExtend.set(0.0)
        '''
        #controls can be reversed if spool spins to much
        if (self.PadX == -1 and self.armLength == 1):
            if (self.PadX == 0):
                self.armExtend.set(0)
            if (self.PadX == -1):
                self.armExtend.set(0.0)
            if (self.PadX == 1):
                self.armExtend.set(0.9)
        '''
        


        #print("X =", self.PadX)
        #print("Y =", self.PadY)
        
        #CONTROLLER BUTTONS
        if self.controller.getAButtonPressed():
            print("Controller button 1 pressed")
            #self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        if self.controller.getXButtonPressed():
            print("Controller button 2 pressed")
            '''
            self.armStopValue0 = self.armStop.getValue()
            self.armExtend.set(-0.9)
            if (self.armLength == 0):
                self.armStopValue1 = self.armStop.getValue()
            elif (self.armLength == 1 and self.armDirection == 1):
                self.armExtend.set(0.0)
            if self.armStopValue1 > ArmStopVar and self.armStopValue1 > self.armStopValue0:
                self.armDirection = 1
            '''
        if self.controller.getBButtonPressed():
            print("Controller button 3 pressed")
            #self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
            '''
            self.armUpDown.set(0.5)
            if (self.armangle < -490):
                self.armUpDown.set(0.0)
            '''
        if self.controller.getYButtonPressed():
            '''
            self.armStopValue0 = self.armStop.getValue()
            self.armExtend.set(0.9)
            if (self.armLength == 0):
                self.armStopValue1 = self.armStop.getValue()
            elif (self.armLength == 1 and self.armDirection == 0):
                self.armExtend.set(0.0)
            if self.armStopValue1 > ArmStopVar and self.armStopValue1 > self.armStopValue0:
                self.armDirection = 0 
            '''
        if self.controller.getRawButtonPressed(5):
            print("Controller button 5 pressed")
            #temp code to raise arm
            #self.armExtend.set(0.9)
        if self.controller.getRawButtonPressed(6):
            print("Controller button 6 pressed")
            #temp code to extend arm
            #switches to peg mode
            #print("Setting to Peg Mode")
            #self.lmtable.putNumber('pipeline', 1)
        if self.controller.getRawButtonPressed(7):
            print("Controller button 7 pressed")
            self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
            #self.clawStatus = 0
            self.sd.putString('Claw', '')
            #switches to regluar camera mode
            #print("Setting to Camera mode")
            #self.lmtable.putNumber('pipeline', 2)
        if self.controller.getRawButtonPressed(8):
            print("Controller button 8 pressed")
            self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
            #self.clawStatus = 1
            self.sd.putString('Claw', ' X')
            #sets to april tag mode
            #print("Setting to April Tag Mode")
            #self.lmtable.putNumber('pipeline', 0)
        if self.controller.getRawButtonPressed(9):
            print("Controller button 9 pressed")
            #turns limelight on
            #self.lmtable.putNumber('ledMode', 3)
        if self.controller.getRawButtonPressed(10):
            print("Controller buttone 10 pressed")
            #turns limelight off
            #self.lmtable.putNumber('ledMode', 1)
    



    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        #self.driveTrain.setSafetyEnabled(True)
        self.driveTrain.setSafetyEnabled(False)
        self.driveTrain.setExpiration(0.1)
        self.driveTrain.feed()
        self.timer.reset()
        self.timer.start()

        """Runs the motors with tank steering"""
        #self.myRobot.tankDrive(self.leftStick.getY() * -1, self.rightStick.getY() * -1)
        i = 0
        #if i==0:
        #    print("In TeleopMode")
        #    i=i+1
        #self.driveTrain.arcadeDrive(-0.5, 0)
        #print("Driving")
        #time.sleep(1)
        #print("Sleep Complete")
        #self.driveTrain.arcadeDrive(0,0)
        #self.myRobot.arcadeDrive(
        #    self.stick.getRawAxis(0), self.stick.getRawAxis(1), True
        #)
        #print(self.joystick.getY())
        #print(self.joystick.getX())
        #print(self.joystick.getRawButtonPressed(1))
        
        #LIMELIGHT Variables
        
        self.tx = self.lmtable.getNumber('tx', None)
        self.ty = self.lmtable.getNumber('ty', None)
        self.ta = self.lmtable.getNumber('ta', None)
        self.ts = self.lmtable.getNumber('ts', None)
        self.displacement = self.navxer.getDisplacementX()
        self.angle = self.navxer.getAngle()
        self.yaw = self.navxer.getYaw()

        self.limelightLensHeightInches = 14

        self.angler = navx.AHRS.create_i2c(wpilib.I2C.Port.kOnboard)

        self.AutoState = 0
        '''
        #target distance to dashborard
        self.distance = targetDistance(self.ta)
        self.sd.putNumber('tDistance', self.distance)

        #target alignment pushing to dashboard
        self.aim = targetAlignment(self.tx)
        self.sd.putString('tAim', self.aim)

        #arm extension to dashboard
        self.reach = armExtension(self.distance)
        self.sd.putNumber('reach', self.reach)
        '''
        print ("InAutomousInit")
    def autonomousPeriodic(self):
        self.timer.start()
        self.tx = self.lmtable.getNumber('tx', None)
        self.ty = self.lmtable.getNumber('ty', None)
        self.ta = self.lmtable.getNumber('ta', None)
        self.ts = self.lmtable.getNumber('ts', None)
        self.displacement = self.navxer.getDisplacementX()
        self.angle = self.navxer.getAngle()
        self.yaw = self.navxer.getYaw()

        self.limelightLensHeightInches = 14

        #target distance to dashborard
        self.distance = targetDistance(self.ta)
        self.sd.putNumber('tDistance', self.distance)

        #set limelight to april tag mode
        self.lmtable.putNumber('pipeline', 0)

        #self.armangle = self.angler.getAngle()

        #get arm postion using encoder
        self.armangle = self.armUpDown.getSelectedSensorPosition(0)

        self.armStop = wpilib.AnalogInput(1)
       
        """This function is called periodically during autonomous."""
        '''
        print("Autonomous Mode")
        self.driveTrain.arcadeDrive(-0.5, 0)
        #print("Driving")
        time.sleep(1)
        #p  rint("Sleep Complete")
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
        #if self.timer.get() < 2.0:
        #   self.driveTrain.arcadeDrive(-0.7, 0)  # Drive forwards at half speed
        #else:
        #    self.driveTrain.arcadeDrive(0, 0)  # Stop robot

        #DISTANCE
        '''
        if (self.distance >= autoGo):
            self.driveTrain.arcadeDrive(-0.7, 0)
        elif (self.distance <= autoStop):
            self.driveTrain.arcadeDrive(0, 0)
            #event.wait(1)
        else:
            self.driveTrain.arcadeDrive(0, 0)
        
        #self.driveTrain.arcadeDrive(x, y); x = forward, back/ y = right, left
        #ALIGNMENT lim
        
        if (self.tx > autoAimLeft):
            self.driveTrain.arcadeDrive(0, 0.6)
            print("tx = ", self.tx)
            #event.wait(0.05)
        elif (self.tx < autoAimRight):
            self.driveTrain.arcadeDrive(0, -0.6)
            print("-tx =", self.tx)
        '''
        
        if (self.AutoState == 0):
            self.armUpDown.set(-0.5)
            event.wait(2)
            self.armUpDown.set(0.0)
            self.AutoState = 1
            self.AutoState1Complete = self.timer.get()
            '''
            if (self.armangle < ArmAngleAutoVar):
                self.armUpDown.set(0.0)
                self.AutoState = 1
                self.AutoState1Complete = self.timer.get()
            '''
        if (self.AutoState == 1):
            self.AutoState2 = self.timer.get() - self.AutoState1Complete
            if self.AutoState2 < 3.0:
                self.armExtend.set(0.5)
            else:
                self.armExtend.set(0.0)
                self.AutoState2Complete = self.timer.get()
                self.AutoState = 2
        if (self.AutoState == 2):
            self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
            event.wait(1)
            self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
            self.AutoState = 3
        if (self.AutoState == 3):
            self.armExtend.set(-0.5)
        elif (self.armExtend == 1):
            self.armExtend.set(0.0)
            self.AutoState = 4
''' 
        if (abs(self.tx) > 1.0):
            if (self.tx < 0):
                steeringAdjust = kp * self.tx + min_command
            else:
                steeringAdjust = kp * self.tx - min_command
            print("steering adj = ", steeringAdjust)
'''        

'''
    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.drive.arcadeDrive(self.stick.getY(), self.stick.getX())
'''

# MAIN LOOP

if __name__ == '__main__':
    print("Working")
    wpilib.run(MyRobot)
   
