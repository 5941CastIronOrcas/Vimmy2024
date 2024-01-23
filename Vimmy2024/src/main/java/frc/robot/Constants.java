// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //public static final Pigeon2 gyro = new Pigeon2(2);
  public static final Pigeon2 gyro = new Pigeon2(2);

  public static final CANSparkMax flaMotor = new CANSparkMax(23, MotorType.kBrushless);
  public static final CANSparkMax fltMotor = new CANSparkMax(22, MotorType.kBrushless);
  public static final CANSparkMax fraMotor = new CANSparkMax(21, MotorType.kBrushless);
  public static final CANSparkMax frtMotor = new CANSparkMax(20, MotorType.kBrushless);
  public static final CANSparkMax blaMotor = new CANSparkMax(25, MotorType.kBrushless);
  public static final CANSparkMax bltMotor = new CANSparkMax(24, MotorType.kBrushless);
  public static final CANSparkMax braMotor = new CANSparkMax(27, MotorType.kBrushless);
  public static final CANSparkMax brtMotor = new CANSparkMax(26, MotorType.kBrushless);

  public static final CANSparkMax armMotor1 = new CANSparkMax(0, MotorType.kBrushless);
  public static final CANSparkMax armMotor2 = new CANSparkMax(0, MotorType.kBrushless);
  public static final Boolean armMotor1Invert = false;
  public static final Boolean armMotor2Invert = false;

  public static final double swerveDriveRatio = 1.00 / 6.75;
  public static final double swerveWheelCircumference = 0.096774 * Math.PI; // in m
  //public static final WPI_CANCoder flEncoder = new WPI_CANCoder(0);
  public static final CANcoder flEncoder = new CANcoder(0);
  public static final CANcoder frEncoder = new CANcoder(0);
  public static final CANcoder blEncoder = new CANcoder(0);
  public static final CANcoder brEncoder = new CANcoder(0);

  public static final double controllerDeadZone = 0.1;
  public static final double modulePMult = 0.02;
  public static final double turnMult = 0.2;
  public static final double maxThrottleChange = 0.1;
  
  public static final double swerveAutoTurnPMult = 0.005;
  public static final double swerveAutoTurnMaxSpeed = 0.5;
  public static final double swerveAutoTurnDeadZone = 0.5;
  
  public static final double swerveDriveToPMult = 1.0;
  public static final double swerveDriveToDMult = 7.0;
  public static final double swerveDriveToMaxSpeed = 3.6576;
  public static final double swerveDriveToDeadZone = 0.01;
  
  public static final XboxController controller1 = new XboxController(0);
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
