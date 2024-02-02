// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;

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

  //Gyro
  public static final Pigeon2 gyro = new Pigeon2(54);

  //Controller Stuff
  public static final XboxController controller1 = new XboxController(0);
  public static final XboxController controller2 = new XboxController(1);
  public static final double controllerDeadZone = 0.1;

  //Swerve Motor Declarations:
  public static final CANSparkMax flaMotor = new CANSparkMax(27, MotorType.kBrushless);
  public static final CANSparkMax fltMotor = new CANSparkMax(26, MotorType.kBrushless);
  public static final CANSparkMax fraMotor = new CANSparkMax(25, MotorType.kBrushless);
  public static final CANSparkMax frtMotor = new CANSparkMax(24, MotorType.kBrushless);
  public static final CANSparkMax blaMotor = new CANSparkMax(21, MotorType.kBrushless);
  public static final CANSparkMax bltMotor = new CANSparkMax(20, MotorType.kBrushless);
  public static final CANSparkMax braMotor = new CANSparkMax(23, MotorType.kBrushless);
  public static final CANSparkMax brtMotor = new CANSparkMax(22, MotorType.kBrushless);
  //Swerve Encoder Declarations
  public static final CANcoder flEncoder = new CANcoder(50);
  public static final CANcoder frEncoder = new CANcoder(51);
  public static final CANcoder blEncoder = new CANcoder(52);
  public static final CANcoder brEncoder = new CANcoder(53);
  //Arm Motor Declarations:
  public static final CANSparkMax armMotor = new CANSparkMax(28, MotorType.kBrushless);
  public static final Boolean armMotorInvert = false;
  public static final CANSparkMax intakeMotor = new CANSparkMax(29, MotorType.kBrushless);
  public static final CANSparkMax shooterMotor1 = new CANSparkMax(30, MotorType.kBrushless);
  public static final CANSparkMax shooterMotor2 = new CANSparkMax(31, MotorType.kBrushless);
  //Limit Switch Declarations
  //public static final ArrayList<DigitalInput> limitSwitches = new ArrayList<DigitalInput>();
  public static final DigitalInput[] limitSwitches = new DigitalInput[]{new DigitalInput(0)};


  //Arm Constants
  public static double armMotorPMult = 1.0/90.0;
  public static double armMotorDMult = 0;
  public static double armMotorGravMult = 0; //how much the arm PID compensates for gravity
  public static double maxArmSpeed = 0.3; //Max speed the arm PID is allowed to output to the arm motor
  public static double armAngleVariation = 1; //how close the arm has to be to the target angle in degrees to allow shooting

  public static final double intakeAngle = 0; //the angle in degrees the arm should be at to intake a ring
  public static final double minShootRpm = 5000; //the minimum RPM the shooter needs to be at to shoot
  public static final double ampDepositAngle = 90; //the angle the arm should be at to do the amp
  public static final Vector2D redSpeaker = new Vector2D(0,0);
  public static final Vector2D blueSpeaker = new Vector2D(0,0);

  //Swerve Module Constants
  public static final double swerveDriveRatio = 1.00 / 6.75; //L2=1/6.75  L3=1/6.12
  public static final double swerveWheelCircumference = 0.096774 * Math.PI; // in m
  public static final double modulePMult = 0.01;
  public static final double maxThrottleChange = 2.0; //the maximum amount the wheel throttle is allowed to change per frame (max 2.0)
  public static final double swerveMaxAccel = 0.3; //the max amount swerve is allowed to accelerate, measured in percent per frame (max 2.0)
  
  public static final double turnMult = 1.0; //the max speed Swerve is EVER allowed to turn at
  public static final double swerveAutoTurnPMult = 0.006;
  public static final double swerveAutoTurnDMult = 0.00035;
  public static final double swerveAutoTurnMaxSpeed = 1.0; //the max speed Swerve is allowed to turn at when turning itself
  public static final double swerveAutoTurnDeadZone = 0.5; //if swerve is pointing within this many degrees of where it wants to point, it stops rotating.
  
  public static final double swerveDriveToPMult = 1.0;
  public static final double swerveDriveToDMult = 7.0;
  public static final double swerveMaxSpeed = 4.60248; //the max speed we're capable of moving at (used to discard impossible data)
  public static final double swerveDriveToDeadZone = 0.03; //if the robot is within this many meters of the target position, it stops moving.
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
