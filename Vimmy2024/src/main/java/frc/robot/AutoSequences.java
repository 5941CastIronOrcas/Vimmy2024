package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoSequences {
  // kill all the motors
  public static void autoSequence0() {
    killAllTheMotors();
  }

  // retract climbers
  public static void autoSequence1() {
    ClimberSubsystem.moveClimbers(-1, 0);
    killAllTheMotors();
  }

  // drive out
  public static void autoSequence2() {
    ClimberSubsystem.moveClimbers(-1, 0);
    if(isAutoTimeBetween(0.0, 3.0)) SwerveSubsystem.DriveDriverOriented(0, 0.25, 0);
    else killAllTheMotors();
  }

  // shoot basic
  public static void autoSequence3() {
    ClimberSubsystem.moveClimbers(-1, 0);
    if (isAutoTimeBetween(0, 2)) {
        ArmSubsystem.PrepShooter(0.8);
    } else if (isAutoTimeBetween(2, 3)) {
        ArmSubsystem.SpinIntake(0.75);
        ArmSubsystem.PrepShooter(0.8);
    }
    else killAllTheMotors();
  }

  //shoot aimbot
  public static void autoSequence4() {
        ClimberSubsystem.moveClimbers(-1, 0);
    if (isAutoTimeBetween(0, 2)) {
        ArmSubsystem.PrepShooter(1);
        ArmSubsystem.ShootSpeaker();
    } else killAllTheMotors();
  }

  // shoot basic, drive out
  public static void autoSequence5() {
        ClimberSubsystem.moveClimbers(-1, 0);
        if (isAutoTimeBetween(0, 3)) {
            SwerveSubsystem.DriveDriverOriented(0, 0.25, 0);
        } else if (isAutoTimeBetween(3, 5)) {
            ArmSubsystem.PrepShooter(1);
            ArmSubsystem.ShootSpeaker();
        } else killAllTheMotors(); 
  }

  // shoot aimbot, drive out
  public static void autoSequence6() {
        ClimberSubsystem.moveClimbers(-1, 0);
    if (isAutoTimeBetween(0, 3)) {
        SwerveSubsystem.DriveDriverOriented(0, 0.25, 0);
    } else if (isAutoTimeBetween(3, 6)) {
        ArmSubsystem.PrepShooter(1);
        ArmSubsystem.ShootSpeaker();
    } else killAllTheMotors();
    
  }

  // shoot basic, collect nearest
  public static void autoSequence7() {
        
  }

  // shoot aimbot, collect nearest
  public static void autoSequence8() {
    
  }

  // shoot basic, collect nearest, shoot
  public static void autoSequence9() {
    
  }

  // shoot aimbot, collect nearest, shoot
  public static void autoSequence10() {
    
  }



  public static boolean isAutoTimeBetween(double timeMin, double timeMax) {
    return timeMin < Timer.getFPGATimestamp() - Constants.timeSinceStartAtAutoStart && timeMax > Timer.getFPGATimestamp() - Constants.timeSinceStartAtAutoStart ;
  }
  public static void killAllTheMotors() {
    Constants.intakeMotor.stopMotor();
    Constants.climberMotorR.stopMotor();
    Constants.climberMotorL.stopMotor();
    Constants.shooterMotor1.stopMotor();
    Constants.shooterMotor2.stopMotor();
    Constants.armMotor1.stopMotor();
    Constants.armMotor2.stopMotor();
    Constants.blaMotor.stopMotor();
    Constants.bltMotor.stopMotor();
    Constants.braMotor.stopMotor();
    Constants.brtMotor.stopMotor();
    Constants.flaMotor.stopMotor();
    Constants.fltMotor.stopMotor();
    Constants.fraMotor.stopMotor();
    Constants.frtMotor.stopMotor();
  }
}
