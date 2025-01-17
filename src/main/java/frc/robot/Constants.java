// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.utilityObjects.Vector2D;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //CONTROLLER STUFF:
  public static final XboxController controller1 = new XboxController(0); //drivetrain controller
  public static final XboxController controller2 = new XboxController(1); //arm controller
  public static final double controllerDeadZone = 0.1; //how far the stick can go without triggering input

  //Misc
  //public static final DigitalInput limpRobotButton = new DigitalInput(0);
  public static final int defaultAutoSequence = 0;

  public static double timeSinceStartAtAutoStart = 0;
  //SWERVE STUFF:
  //Gyro
  public static final Pigeon2 gyro = new Pigeon2(54); 
  //Swerve Motor Declarations
  public static final CANSparkMax flaMotor = new CANSparkMax(27, MotorType.kBrushless); //front left angle motor
  public static final CANSparkMax fltMotor = new CANSparkMax(26, MotorType.kBrushless); //front left throttle motor
  public static final CANSparkMax fraMotor = new CANSparkMax(25, MotorType.kBrushless); //front right angle motor
  public static final CANSparkMax frtMotor = new CANSparkMax(24, MotorType.kBrushless); //front right throttle motor
  public static final CANSparkMax blaMotor = new CANSparkMax(21, MotorType.kBrushless); //back left angle motor
  public static final CANSparkMax bltMotor = new CANSparkMax(20, MotorType.kBrushless); //back left throttle motor
  public static final CANSparkMax braMotor = new CANSparkMax(23, MotorType.kBrushless); //back right angle motor
  public static final CANSparkMax brtMotor = new CANSparkMax(22, MotorType.kBrushless); //back right throttle motor
  //Swerve Motor Inversions
  public static final boolean fltInvert = true;
  public static final boolean frtInvert = false;
  public static final boolean bltInvert = false;
  public static final boolean brtInvert = false;
  //Swerve Encoder Declarations
  public static final CANcoder flEncoder = new CANcoder(50);
  public static final CANcoder frEncoder = new CANcoder(51);
  public static final CANcoder blEncoder = new CANcoder(52);
  public static final CANcoder brEncoder = new CANcoder(53);
  //Swerve Module Constants
  public static final double swerveDriveRatio = 1.00 / 6.75; //L2=1/6.75  L3=1/6.12
  public static final double swerveWheelCircumference = 0.096774 * Math.PI; // in m
  public static final double modulePMult = 0.01;
  public static final double maxThrottleChange = 2.0; //the maximum amount the wheel throttle of each module is allowed to change per frame (max 2.0)
  public static final double swerveMaxAccel = 2.0; //the max amount swerve is allowed to accelerate, measured in percent per frame (max 2.0)
  public static final double swerveMaxAccelExtended = 0.6; //the max amount swerve is allowed to accelerate when the arm is fully extended
  //Swerve Drive Turning Constants
  public static final double turnMult = 1.0; //the max speed Swerve is EVER allowed to turn at
  public static final double swerveAutoTurnPMult = 0.007;
  public static final double swerveAutoTurnDMult = 0.0005;
  public static final double swerveAutoTurnMaxSpeed = 1.0; //the max speed Swerve is allowed to turn at when turning itself
  public static final double swerveAutoTurnDeadZone = 2.0; //if swerve is pointing within this many degrees of where it wants to point, it stops rotating.
  public static final double swerveAngleVariation = 1.0; //swerve has to be within this many degrees of the right direction for the shooter to shoot
  //Swerve Drive To Constants
  public static final double swerveDriveToPMult = 1.0;
  public static final double swerveDriveToDMult = 0.14;
  public static final double swerveDriveToDeadZone = 0.03; //if the robot is within this many meters of the target position, it stops moving.
  public static final double swerveSquareDriveToDeadZone = 0.03;
  //Swerve Collect Ring Constants
  public static final double swerveCollectNotePMult = 0.5;
  
  
  //GOA CONSTANTS:
  //Object-Specific Avoidance Mults
  public static final double subwooferAvoidanceMult = 1.0;
  public static final double stageAvoidanceMult = 1.0;
  //General Settings:
  public static final double avoidanceMult = 2.0;
  public static final double VelocityAvoidanceMult = 4.0;
  public static final double GOAIgnoreRange = 5.0;
  public static final double avoidanceExponent = 4.0;
  public static final double toTargetStrengthTop = 2.0;
  public static final double toTargetStrengthCap = 5.0;
  //SonarModules
  public static final Servo servo1 = new Servo(1);
  public static final Servo servo2 = new Servo(2);
  public static final Servo servo3 = new Servo(3);
  public static final Servo servo4 = new Servo(4);
  public static final Servo servo5 = new Servo(5);
  public static final Servo servo6 = new Servo(6);
  public static final Servo servo7 = new Servo(7);
  public static final Servo servo8 = new Servo(8);
  public static AnalogInput ultrasonicSensor1 = new AnalogInput(0);
  public static AnalogInput ultrasonicSensor2 = new AnalogInput(1);
  public static AnalogInput ultrasonicSensor3 = new AnalogInput(2);
  public static AnalogInput ultrasonicSensor4 = new AnalogInput(3);
  public static AnalogInput ultrasonicSensor5 = new AnalogInput(4);
  public static AnalogInput ultrasonicSensor6 = new AnalogInput(5);
  public static AnalogInput ultrasonicSensor7 = new AnalogInput(6);
  public static AnalogInput ultrasonicSensor8 = new AnalogInput(7);
  public static DigitalInput linebreakSensor = new DigitalInput(9);
  public static DigitalOutput ultrasonicPing = new DigitalOutput(0);
  public static double voltageScaleFactor = 0;
  //Pan the servos 
  public static final boolean isSonarModulesEnabled = true; //true - code will process, false - code will not process, servos will not pan
  public static final double ServoSpeed = 0.15; // changes how fast will the sine wave will go
  //GarbageDataCollector
  public static final double SonicMaxValue = 500; // changes the max and min values for value filter
  public static final double SonicMinValue = 30;
  public static final double servoFinalMult = 180.0 / 210.0;

  //GOA data tune
  public static final short maxObstacleLifeFrames = 15;
  public static final short GOAFrameRate = 5;

  // is GOA enabled
  public static final boolean isGOASynthEnabled = false;


  //ARM STUFF:

  //Arm Motor Declarations
  public static final CANSparkMax armMotor1 = new CANSparkMax(28, MotorType.kBrushless); //left arm motor viewing from the front of the robot
  public static final CANSparkMax armMotor2 = new CANSparkMax( 29, MotorType.kBrushless); //right arm motor
  public static final Boolean armMotor1Invert = true;
  public static final Boolean armMotor2Invert = false;
  public static final CANSparkMax intakeMotor = new CANSparkMax(30, MotorType.kBrushless);
  public static final CANSparkMax lowerShooterMotor = new CANSparkMax(31, MotorType.kBrushless);
  public static final CANSparkMax upperShooterMotor = new CANSparkMax(32, MotorType.kBrushless);
  //Arm Sensor Declarations
  public static DutyCycleEncoder armJointEncoder = new DutyCycleEncoder(1);

  //public static final DigitalInput[] noteDetectionSwitches = new DigitalInput[]{new DigitalInput(0)};
  //Arm Control Constants
  public static final double armMotorPMult = 1.0/10.0;
  public static final double armMotorDMult = 0.0;
  public static final double armMotorGravMult = 0.02; //how much the arm PID compensates for gravity
  public static final double maxArmSpeed = 1.0; //Max speed the arm PID is allowed to output to the arm motor
  public static final double armAngleVariation = 1.0; //how close the arm has to be to the target angle in degrees to allow shooting
  public static final double armAngleOffset = 60;
  public static final double minArmAngle = 2;
  public static final double maxArmAngle = 99;
  //Intake / Shooter Control Constants
  public static final double minShootRpm = 5000; //the minimum RPM the shooter needs to be at to shoot
  public static final double intakeAngle = 0.0; //the angle in degrees the arm should be at to intake a ring
  public static final double ampDepositAngle = 99.0; //the angle the arm should be at to do the amp
  public static final double trapShootAngle = 30;
  public static final double trapShootSpeed = 0.5;
  public static final double trapMinRPM = 2000;
  public static final double launchSpeed = 10.0; //speed of ring after being launched in m/s
  public static final double gravity = 9.81; //gravity acceleration in m/s^2
  public static final double defaultShooterSpeed = 1.0;
  public static final double bottomRpmMult = 1.0;
  public static final double shootYawOffset = 3.5; //change if note doesn't shoot straight out of the shooter           <----HERE'S THE VARIABLE
  public static final double maxShootingRange = 3.5;
  // theshold for sonic sensor
  public static final double hasNoteTreshold = 10;
  //CLIMBER STUFF
  //Climber Motor Declarations
  public static final CANSparkMax climberMotorL = new CANSparkMax(33, MotorType.kBrushless);
  public static final CANSparkMax climberMotorR = new CANSparkMax(34, MotorType.kBrushless);
  public static DigitalInput lClimberSwitch = new DigitalInput(2);
  public static DigitalInput rClimberSwitch = new DigitalInput(3);
  public static DigitalInput tesDigitalInputHUH = new DigitalInput(7);
  public static final Boolean climberMotorLInvert = false;
  public static final Boolean climberMotorRInvert = false;
  //Climber Control Constants
  public static final double climberBalancePMult = 0.01;
  public static final double climberBalanceDMult = 0.01;
  public static final double climberMaxHitSpeed = 0.5;
  public static final double climberSmoothingStart = 20;
  public static final double climberSmoothingEnd = 10;
  public static final double climberReductionMult = (climberMaxHitSpeed-1) / (climberSmoothingStart-climberSmoothingEnd);
  //public static final double climberMaxSpeed = 1;
  public static final double climberMaxHeight = 95;
  public static final double climberGoToPMult = 0.2;

  //POSITION ESTIMATION AND FIELD CONSTANTS:
  public static final Vector2D redSpeaker = new Vector2D(16.579342, 5.547868);
  public static final Vector2D blueSpeaker = new Vector2D(-0.0381, 5.547868 );
  public static final Vector2D redAmpDepositPosition = new Vector2D(14.700758, 8.24); //7.74
  public static final Vector2D blueAmpDepositPosition = new Vector2D(1.8415, 8.24);
  public static final double speakerHeight = 2.05; //height of speaker opening in meters
  public static final double speakerAngleVariation = 5.0; //how many degrees the arm angle can be from the target and still shoot
  public static final double noteCameraHeight = 0.22; //in meters for note detector
  public static final double noteCameraForwardOffset = 0.54; // forward distance from robot center to note detector camera in meters
  public static final double noteCameraAngle = -20; //for note detector
  public static final double swerveMaxSpeed = 4.4; //the max speed we're capable of moving at in m/s (used for discarding impossible data)
  public static final String apriltagCamera1Name = "Arducam_OV9281_USB_Camera 1"; //LEFT (shooter forward)
  public static final String apriltagCamera2Name = "Arducam_OV9281_USB_Camera 2"; //RIGHT (shooter forward)
  public static final String noteDetectionCameraName = "Arducam_OV9782_USB_Camera";
  public static final double FieldDisplayOffsetX = 1.1225;
  public static final double FieldDisplayOffsetY = 0.326;



  public static final Vector2D[] blueNotesPos = new Vector2D[] {new Vector2D(2.9464, 4.1057), new Vector2D(2.9464, 5.5535), new Vector2D(2.9464, 7.0013)}; 
  public static final Vector2D[] redNotesPos = new Vector2D[] {new Vector2D(13.6449, 4.1057), new Vector2D(13.6449, 5.5535), new Vector2D(13.6449, 7.0013)};
  public static final Vector2D[] centerNotesPos = new Vector2D[] {new Vector2D(8.2956, 0.7529), new Vector2D(8.2956, 2.4293), new Vector2D(8.2956, 4.1057), new Vector2D(8.2956, 5.7821), new Vector2D(8.2956, 7.4585)};
  public static Vector2D[] allNotesPos = new Vector2D[centerNotesPos.length + redNotesPos.length];
  
  //ANSI Color Codes:
  public static final String ansiRESET = "\u001B[0m";
  public static final String ansiBLK = "\u001B[30m";
  public static final String ansiRED = "\u001B[31m";
  public static final String ansiGRN = "\u001B[32m";
  public static final String ansiYLW = "\u001B[33m";
  public static final String ansiBLU = "\u001B[34m";
  public static final String ansiPRP = "\u001B[35m";
  public static final String ansiCYN = "\u001B[36m";
  public static final String ansiWHT = "\u001B[37m";
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  
  }
}
