// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.constant.DirectMethodHandleDesc;
import java.sql.Driver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArduinoCommunication;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriverDisplay;
import frc.robot.subsystems.GOAGuidanceSystem;
import frc.robot.subsystems.PositionEstimator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilityObjects.Vector2D;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static Boolean isRedAlliance = true;
  public static Boolean isBlueAlliance = false;
  public static boolean robotLimp = true;
  public static int selectedAutoSequence = Constants.defaultAutoSequence;
  //public static boolean limpButtonOld = Constants.limpRobotButton.get();

  private RobotContainer m_robotContainer;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    ArduinoCommunication.Wrap();
    Constants.climberMotorL.getEncoder().setPosition(0);
    Constants.climberMotorR.getEncoder().setPosition(0);
    Constants.gyro.setYaw(180);
     
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    isRedAlliance = DriverStation.getAlliance().toString().equals("Optional[Red]");
    isBlueAlliance = DriverStation.getAlliance().toString().equals("Optional[Blue]");
    selectedAutoSequence = (int)DriverDisplay.AutoSequence.getInteger(Constants.defaultAutoSequence);
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  //@Override
  /*public void disabledPeriodic() {
    if(!limpButtonOld && Constants.limpRobotButton.get())
    {
      robotLimp = !robotLimp;
      Functions.setRobotLimp(robotLimp);
    }
    limpButtonOld = Constants.limpRobotButton.get();
  }*/

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      robotLimp = false;
      Functions.setRobotLimp(robotLimp);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { 
    double speed = 1-(0.75*Constants.controller1.getLeftTriggerAxis());
    double LSX = Functions.Exponential(Functions.DeadZone(Constants.controller1.getLeftX(), Constants.controllerDeadZone)) * speed;
    double LSY = -Functions.Exponential(Functions.DeadZone(Constants.controller1.getLeftY(), Constants.controllerDeadZone)) * speed;
    double RSX = Functions.Exponential(Functions.DeadZone(Constants.controller1.getRightX(), Constants.controllerDeadZone)) * speed;
    double RSY = -Functions.Exponential(Functions.DeadZone(Constants.controller1.getRightY(), Constants.controllerDeadZone)) * speed;
    double LSY2 = -Functions.Exponential(Functions.DeadZone(Constants.controller2.getLeftY(), Constants.controllerDeadZone));
    double RSY2 = -Functions.Exponential(Functions.DeadZone(Constants.controller2.getRightY(), Constants.controllerDeadZone));
    double RSX2 = Functions.Exponential(Functions.DeadZone(Constants.controller2.getRightX(), Constants.controllerDeadZone));
    double RSAngle = 90-Math.toDegrees(Math.atan2(RSY, RSX));

    if (Constants.controller1.getRightBumperPressed()) {
      Constants.gyro.setYaw(180);
    }
    //SwerveSubsystem.Drive(LSX, LSY, RSX);
    //SwerveSubsystem.DriveDriverOriented(LSX, LSY, RSX);
    //SwerveSubsystem.DriveFieldOriented(LSX, LSY, RSX);

    //Swerve
    if(Constants.controller1.getBButton())
    {
      SwerveSubsystem.CollectNote(LSX, LSY, speed);
    }
    else if(Constants.controller1.getXButton())
    {
      SwerveSubsystem.FaceSpeaker(LSX, LSY, speed);
    }
    else if(Constants.controller1.getYButton())
    {
      SwerveSubsystem.GoToAmp(speed, speed, LSX, LSY);
    }
    else if(Constants.controller1.getAButton())
    {
      SwerveSubsystem.DriveDriverOrientedProtectIntake(LSX, LSY, speed);
    }
    else if(Constants.controller1.getPOV() == 90) {SwerveSubsystem.DriveDriverOrientedAtAngle(LSX, LSY, -60, speed);}
    else if(Constants.controller1.getPOV() == 270) {SwerveSubsystem.DriveDriverOrientedAtAngle(LSX, LSY, 60, speed);}
    else if(Constants.controller1.getPOV() == 0) {SwerveSubsystem.DriveDriverOrientedAtAngle(LSX, LSY, 0, speed);}
    else
    {
      SwerveSubsystem.DriveDriverOrientedAtAngle(LSX,LSY,RSAngle+180,Functions.Pythagorean(RSX, RSY));
    }

    //Arm
    if(Constants.controller1.getBButton())
    {
      ArmSubsystem.IntakeRing();
    }
    else if(Constants.controller1.getXButton())
    {
      ArmSubsystem.PrepShooter(Constants.defaultShooterSpeed);
      ArmSubsystem.ShootSpeaker();
    }
    else if(Constants.controller1.getRightTriggerAxis() > 0.5)
    {
      ArmSubsystem.IntakeRing();
    }
    else
    {
      if(Constants.controller2.getYButton())
      {
        ArmSubsystem.DepositAmp();
      }
      else
      {
        if(Constants.controller2.getRightBumper())
        {
          ArmSubsystem.moveArmTo(16.1);
        }
        else
        {
          ArmSubsystem.rotateArm(LSY2);
        }
        
        if(Constants.controller2.getRightBumper())
        {
          ArmSubsystem.SpinShooter(1);
        }
        else
        {
          ArmSubsystem.SpinShooter(Constants.controller2.getRightTriggerAxis()-Constants.controller2.getLeftTriggerAxis());
        }
      }
      if(Constants.controller2.getXButton())
        {
          ArmSubsystem.SpinIntake(0.5);
        }
        else if(Constants.controller2.getBButton())
        {
          ArmSubsystem.SpinIntake(-0.25);
        } else if (Constants.controller2.getAButton()) {
          ArmSubsystem.Intake(0.5);
        }
        else
        {
          ArmSubsystem.SpinIntake(-Constants.controller2.getLeftTriggerAxis());
        }
      
    }

    ClimberSubsystem.moveClimbers(RSY2, RSX2);
    
  }


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}


  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    for (int i = 0; i < Constants.allNotesPos.length; i++) PositionEstimator.realNoteList.add(Constants.allNotesPos[i]);
    Constants.timeSinceStartAtAutoStart = Timer.getFPGATimestamp();
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    AutoSequences.AutoStart();

  }


  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    switch(selectedAutoSequence) {
      case 0:
        AutoSequences.autoSequence0();
        break;
      case 1:
        AutoSequences.autoSequence1();
        break;
      case 2:
        AutoSequences.autoSequence2();
        break;
      case 3:
        AutoSequences.autoSequence3();
        break;
      case 4:
        AutoSequences.autoSequence4();
        break;
      case 5:
        AutoSequences.autoSequence5();
        break;
      case 6:
        AutoSequences.autoSequence6();
        break;
      case 7:
        AutoSequences.autoSequence7();
        break;
      case 8:
        AutoSequences.autoSequence8();
        break;
      case 9:
        AutoSequences.autoSequence9();
        break;
      case 10:
        AutoSequences.autoSequence10();
        break;
      default:
        AutoSequences.autoSequence0();
        break;
    }
  }  
  public static boolean noteIgnoranceGetInpt(String inptStr) {
    return inptStr.toLowerCase().equals("n") || inptStr.toLowerCase().equals("no") || inptStr.toLowerCase().equals("none") || inptStr.toLowerCase().equals("0");
  }
  /*
  public static String BoolToHexString(boolean[] inptData) {
    if (inptData[0]) {
      if (inptData[1]) {
        if (inptData[2]) {
          if (inptData[3]) {

          } else {
            
          }
        } else {
          if (inptData[3]) {

          } else {
            
          }
        }
      } else {
        if (inptData[2]) {
          if (inptData[3]) {

          } else {
            
          }
        } else {
          if (inptData[3]) {

          } else {
            
          }
        }
      }
    } else {
      if (inptData[1]) {
        if (inptData[2]) {
          if (inptData[3]) {

          } else {
            
          }
        } else {
          if (inptData[3]) {

          } else {
            
          }
        }
      } else {
        if (inptData[2]) {
          if (inptData[3]) {

          } else {
            
          }
        } else {
          if (inptData[3]) {

          } else {
            
          }
        }
      }
    }
    return "0x" + ;
  } */
}
