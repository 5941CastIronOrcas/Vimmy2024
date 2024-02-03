// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PositionEstimator;
import frc.robot.subsystems.SwerveSubsystem;

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
  public static double tempDemoAngle = 0; //remove later

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
    SmartDashboard.putNumber("Robot X", PositionEstimator.robotPosition.getX());
    SmartDashboard.putNumber("Robot Y", PositionEstimator.robotPosition.getY());
    SmartDashboard.putNumber("DriverYaw", PositionEstimator.robotYawDriverRelative);
    SmartDashboard.putNumber("FieldYaw", Functions.DeltaAngleDeg(0, PositionEstimator.robotPosition.getRotation().getDegrees()));
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double speed = 1-(0.75*Constants.controller1.getLeftTriggerAxis());
    double LSX = -Functions.Exponential(Functions.DeadZone(Constants.controller1.getLeftX(), Constants.controllerDeadZone)) * speed;
    double LSY = Functions.Exponential(Functions.DeadZone(Constants.controller1.getLeftY(), Constants.controllerDeadZone)) * speed;
    double RSX = Functions.Exponential(Functions.DeadZone(Constants.controller1.getRightX(), Constants.controllerDeadZone)) * speed;
    double RSY = Functions.Exponential(Functions.DeadZone(Constants.controller1.getRightY(), Constants.controllerDeadZone)) * speed;
    double LSY2 = Functions.Exponential(Functions.DeadZone(Constants.controller2.getLeftY(), Constants.controllerDeadZone)) * speed;
    double RSAngle = Math.toDegrees(Math.atan2(RSY, RSX))+90;

    if (Constants.controller1.getRightBumperPressed()) {
      Constants.gyro.setYaw(0);
    }
    if(Constants.controller1.getYButtonPressed())
    {
      PositionEstimator.robotPosition = new Pose2d(0,0,PositionEstimator.robotPosition.getRotation());
      tempDemoAngle = PositionEstimator.robotPosition.getRotation().getDegrees();
    }
    //SwerveSubsystem.Drive(LSX, LSY, RSX);
    //SwerveSubsystem.DriveDriverOriented(LSX, LSY, RSX);
    if(Constants.controller1.getAButton())
    {
      SwerveSubsystem.DriveTo(0, 0, tempDemoAngle, speed, speed);
    }
    else
    {
      SwerveSubsystem.DriveDriverOrientedAtAngle(LSX,LSY,RSAngle,Functions.Pythagorean(RSX, RSY));
    }

    ArmSubsystem.rotateArm(LSY2);
    
    if(Constants.controller2.getXButton())
    {
      ArmSubsystem.SpinIntake(0.75);
    }
    else if(Constants.controller2.getBButton())
    {
      ArmSubsystem.SpinIntake(-0.25);
    }
    else
    {
      ArmSubsystem.SpinIntake(0);
    }
    ArmSubsystem.SpinShooter(Constants.controller2.getRightTriggerAxis());
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
}
