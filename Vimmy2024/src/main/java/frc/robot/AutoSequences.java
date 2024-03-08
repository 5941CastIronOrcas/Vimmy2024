package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriverDisplay;
import frc.robot.subsystems.NoteDetector;
import frc.robot.subsystems.PositionEstimator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilityObjects.Vector2D;

public class AutoSequences {
  public static boolean[] notesIncluded = new boolean[8];

  public static void AutoStart() {
    switch ((int)DriverDisplay.AutoSequence.getInteger(0)) {
      case 0:
        Constants.gyro.setYaw(180);
        DriverDisplay.AutoSequenceDisplay.setString("0 Angle Selected (Intake Forward)");
        break;

      case 1:
        Constants.gyro.setYaw(-120);
        DriverDisplay.AutoSequenceDisplay.setString("Right Subwoofer Side Selected");
        break;

      case 2:
        Constants.gyro.setYaw(120);
        DriverDisplay.AutoSequenceDisplay.setString("Left Subwoofer Side Selected");
        break;

      default:
      Constants.gyro.setYaw(180);
      DriverDisplay.AutoSequenceDisplay.setString("INVALID VALUE");
        break;
    }
    for (int i = 0; i < Constants.redNotesPos.length; i++) Constants.allNotesPos[i] = Robot.isRedAlliance ? Constants.redNotesPos[i] : Constants.blueNotesPos[i];
    for (int i = 0; i < Constants.centerNotesPos.length; i++) Constants.allNotesPos[i + Constants.redNotesPos.length] = Constants.centerNotesPos[i];
    for (int i = 0; i < Constants.allNotesPos.length; i++) if (AutoSequences.notesIncluded[i]) PositionEstimator.realNoteList.add(Constants.allNotesPos[i]);
  }

  // kill all the motors
    public static void autoSequence0() {
        Functions.killAllMotors();
    }

  // retract climbers
    public static void autoSequence1() {
        if (isAutoTimeBetween(0, 3)) ClimberSubsystem.moveClimbers(-1, 0);
        else Functions.killAllMotors();
    }

  // drive out
    public static void autoSequence2() {
        ClimberSubsystem.moveClimbers(-1, 0);
        if(isAutoTimeBetween(0.0, 1.5)) SwerveSubsystem.DriveDriverOriented(0, 0.25, 0);
        else Functions.killAllMotors();
    }

  // shoot basic
    public static void autoSequence3() {
        ClimberSubsystem.moveClimbers(-1, 0);
        if (isAutoTimeBetween(0, 1)) {
            ArmSubsystem.PrepShooter(0.8);
        } else if (isAutoTimeBetween(1, 2)) {
            ArmSubsystem.SpinIntake(0.75);
            ArmSubsystem.PrepShooter(0.8);
        }
        else Functions.killAllMotors();
    }

  //shoot aimbot
    public static void autoSequence4() {
        ClimberSubsystem.moveClimbers(-1, 0);
        if (isAutoTimeBetween(0, 2)) {
            ArmSubsystem.PrepShooter(Constants.defaultShooterSpeed);
            ArmSubsystem.ShootSpeaker();
            SwerveSubsystem.FaceSpeaker(0, 0, 0.25);
        } else Functions.killAllMotors();
    }

  // shoot basic, drive out
    public static void autoSequence5() {
        ClimberSubsystem.moveClimbers(-1, 0);
        if (isAutoTimeBetween(0, 2)) ArmSubsystem.PrepShooter(Constants.defaultShooterSpeed);
        else if (isAutoTimeBetween(2, 3)) {
            SwerveSubsystem.DriveDriverOriented(0, 0, 0);
            ArmSubsystem.PrepShooter(Constants.defaultShooterSpeed);
            ArmSubsystem.SpinIntake(0.75);
        } else if (isAutoTimeBetween(3, 4.5)) {
          ArmSubsystem.SpinIntake(0);
          ArmSubsystem.SpinShooter(0);
          ArmSubsystem.rotateArm(0);
            SwerveSubsystem.DriveDriverOriented(0, 0.25, 0);
        } else Functions.killAllMotors(); 
    }

  // drive out, shoot aimbot
    public static void autoSequence6() {
        ClimberSubsystem.moveClimbers(-1, 0);
    if (isAutoTimeBetween(0, 1.5)) {
        SwerveSubsystem.DriveDriverOriented(0, 0.25, 0);
    } else if (isAutoTimeBetween(1.5, 4.5)) {
        SwerveSubsystem.FaceSpeaker(0, 0, 0.25);
        ArmSubsystem.PrepShooter(Constants.defaultShooterSpeed);
        ArmSubsystem.ShootSpeaker();
    } else Functions.killAllMotors();
    
    }

  // shoot basic, collect nearest
    public static void autoSequence7() {
    boolean wasInDriveToNote = false;
    boolean hadNote = false;
    byte succesfulShots = 0;
      if (isAutoTimeBetween(0, 15) && succesfulShots < 2) {
        ClimberSubsystem.moveClimbers(-1, 0);
        ArmSubsystem.SpinShooter(1);
        if (ArmSubsystem.hasNote) {
          if (wasInDriveToNote) PositionEstimator.removeClosestNote();
            wasInDriveToNote = false;
          hadNote = true;
          ArmSubsystem.PrepShooter(Constants.defaultShooterSpeed);
          if (PositionEstimator.distToSpeaker() < Constants.maxAutoShootingRange) {
            ArmSubsystem.ShootSpeaker();
          } else  {
            if (succesfulShots == 0) {
              SwerveSubsystem.DriveTo((Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).x, (Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).y, PositionEstimator.angleToSpeaker(), 0.5, 0.5, 0, 0);

            } else if (succesfulShots == 1) {
              SwerveSubsystem.DriveDriverOriented(0, 0, 0);
            }
          }

        } else {
          if (hadNote) succesfulShots++; 
          hadNote = false;
          if (NoteDetector.noteVisible) {
            ArmSubsystem.IntakeRing();
            SwerveSubsystem.CollectNote(0, 0, 0.5);        
          } else {
            wasInDriveToNote = true;
            Vector2D closestNote = Constants.allNotesPos[PositionEstimator.nearestAutoNote()];
            if (SwerveSubsystem.atTargetAngle) SwerveSubsystem.DriveTo(closestNote.x, closestNote.y, PositionEstimator.angleToClosestNote(), 0.5, 0.5, 0, 0);
            else SwerveSubsystem.DriveDriverOrientedAtAngle(0, 0, PositionEstimator.angleToClosestNote(), 0.5); 
          }
        }
      } else {
        Functions.killAllMotors();
      }
    }

  // shoot aimbot, collect nearest
  public static void autoSequence8() {
    boolean wasInDriveToNote = false;
    boolean hadNote = false;
    byte succesfulShots = 0;
      if (isAutoTimeBetween(0, 15) && succesfulShots < 2) {
        ClimberSubsystem.moveClimbers(-1, 0);
        ArmSubsystem.SpinShooter(1);

        if (ArmSubsystem.hasNote) {
          if (wasInDriveToNote) PositionEstimator.removeClosestNote();
            wasInDriveToNote = false;
          hadNote = true;
          ArmSubsystem.PrepShooter(1);
          if (PositionEstimator.distToSpeaker() < Constants.maxAutoShootingRange) {
            SwerveSubsystem.DriveDriverOriented(0, 0, 0);
            ArmSubsystem.ShootSpeaker();
          }
          else  {
            SwerveSubsystem.DriveTo((Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).x, (Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).y, PositionEstimator.angleToSpeaker(), 0.5, 0.5, 0, 0);
          }
          
        } else {
          if (hadNote) succesfulShots++; 
          hadNote = false;
          if (NoteDetector.noteVisible) {
            ArmSubsystem.IntakeRing();
            SwerveSubsystem.CollectNote(0, 0, 0.5);        
          } else {
            wasInDriveToNote = true;
            Vector2D closestNote = Constants.allNotesPos[PositionEstimator.nearestAutoNote()];
            if (SwerveSubsystem.atTargetAngle) SwerveSubsystem.DriveTo(closestNote.x, closestNote.y, PositionEstimator.angleToClosestNote(), 0.5, 0.5, 0, 0);
            else SwerveSubsystem.DriveDriverOrientedAtAngle(0, 0, PositionEstimator.angleToClosestNote(), 0.5); 
          }
        }
      } else {
        Functions.killAllMotors();
      }
    
  }

  // shoot basic, collect nearest, shoot
  public static void autoSequence9() {
    boolean wasInDriveToNote = false;
    boolean hadNote = false;
    byte succesfulShots = 0;
      if (isAutoTimeBetween(0, 15)) {
        ClimberSubsystem.moveClimbers(-1, 0);
        ArmSubsystem.SpinShooter(1);
        if (ArmSubsystem.hasNote) {
          if (wasInDriveToNote) PositionEstimator.removeClosestNote();
            wasInDriveToNote = false;
          hadNote = true;
          ArmSubsystem.PrepShooter(1);
          if (PositionEstimator.distToSpeaker() < Constants.maxAutoShootingRange) {
            ArmSubsystem.ShootSpeaker();
          }
          else  {
            if (succesfulShots == 0) {
              SwerveSubsystem.DriveDriverOriented(0, 0, 0);
            } else if (succesfulShots == 1) {
              SwerveSubsystem.DriveTo((Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).x, (Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).y, PositionEstimator.angleToSpeaker(), 0.5, 0.5, 0, 0);
            }         
          }

        } else {
          if (hadNote) succesfulShots++; 
          hadNote = false;
          if (NoteDetector.noteVisible) {
            ArmSubsystem.IntakeRing();
            SwerveSubsystem.CollectNote(0, 0, 0.5);        
          } else {
            wasInDriveToNote = true;
            Vector2D closestNote = Constants.allNotesPos[PositionEstimator.nearestAutoNote()];
            if (SwerveSubsystem.atTargetAngle) SwerveSubsystem.DriveTo(closestNote.x, closestNote.y, PositionEstimator.angleToClosestNote(), 0.5, 0.5, 0, 0);
            else SwerveSubsystem.DriveDriverOrientedAtAngle(0, 0, PositionEstimator.angleToClosestNote(), 0.5); 
          }
        }
      } else {
        Functions.killAllMotors();
      }
  }

  // shoot aimbot, collect nearest, shoot
  public static void autoSequence10() {
    boolean wasInDriveToNote = false;
    boolean hadNote = false;
    byte succesfulShots = 0;
      if (isAutoTimeBetween(0, 15)) {
        ClimberSubsystem.moveClimbers(-1, 0);
        ArmSubsystem.SpinShooter(1);
        if (ArmSubsystem.hasNote) {
          if (wasInDriveToNote) PositionEstimator.removeClosestNote();
          wasInDriveToNote = false;
          if (PositionEstimator.distToSpeaker() < Constants.maxAutoShootingRange) {
            SwerveSubsystem.FaceSpeaker(0, 0, 1);
            ArmSubsystem.PrepShooter(1);
            ArmSubsystem.ShootSpeaker2();
          }
          else  {
            SwerveSubsystem.DriveTo((Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).x, (Robot.isRedAlliance ? Constants.redSpeaker : Constants.blueSpeaker).y, PositionEstimator.angleToSpeaker(), 0.5, 0.5, 0, 0);
            ArmSubsystem.SpinIntake(0);
          }
          
        } else {
          if (hadNote) succesfulShots++; 
          hadNote = false;
          if (NoteDetector.noteVisible) {
            ArmSubsystem.IntakeRing();
            SwerveSubsystem.CollectNote(0, 0, 0.5);        
          } else {
            wasInDriveToNote = true;
            Vector2D closestNote = Constants.allNotesPos[PositionEstimator.nearestAutoNote()];
            if (SwerveSubsystem.atTargetAngle) SwerveSubsystem.DriveTo(closestNote.x, closestNote.y, PositionEstimator.angleToClosestNote(), 0.5, 0.5, 0, 0);
            else SwerveSubsystem.DriveDriverOrientedAtAngle(0, 0, PositionEstimator.angleToClosestNote(), 0.5); 
          }
        }
      } else {
        Functions.killAllMotors();
      }
    
  }



  public static boolean isAutoTimeBetween(double timeMin, double timeMax) {
    return timeMin < Timer.getFPGATimestamp() - Constants.timeSinceStartAtAutoStart && timeMax > Timer.getFPGATimestamp() - Constants.timeSinceStartAtAutoStart ;
  }
  
}
 