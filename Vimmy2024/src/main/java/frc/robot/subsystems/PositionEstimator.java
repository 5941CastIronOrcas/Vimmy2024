// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import org.ejml.equation.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;

public class PositionEstimator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static double robotYawDriverRelative = 0;
  public static Pose2d robotPosition = new Pose2d();  
  public static double deltaX = 0;
  public static double deltaY = 0;
  public PositionEstimator() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    robotYawDriverRelative = Functions.DeltaAngleDeg(0, -Constants.gyro.getYaw().getValueAsDouble());
    if(DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
      robotPosition = new Pose2d(robotPosition.getX(), robotPosition.getY(), new Rotation2d(Functions.DeltaAngleDeg(0, robotYawDriverRelative - 90)));
    } else if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
            robotPosition = new Pose2d(robotPosition.getX(), robotPosition.getY(), new Rotation2d(Functions.DeltaAngleDeg(0, robotYawDriverRelative + 90)));      
    } else {
      robotPosition = new Pose2d(robotPosition.getX(), robotPosition.getY(), new Rotation2d(Functions.DeltaAngleDeg(0, robotYawDriverRelative)));
      
    }
     deltaX = ((
         (Math.sin(Math.toRadians(SwerveSubsystem.frModule.anglePos + robotPosition.getRotation().getDegrees())) * Constants.frtMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference)
       + (Math.sin(Math.toRadians(SwerveSubsystem.flModule.anglePos + robotPosition.getRotation().getDegrees())) * -Constants.fltMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference)
       + (Math.sin(Math.toRadians(SwerveSubsystem.brModule.anglePos + robotPosition.getRotation().getDegrees())) * Constants.brtMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference)
       + (Math.sin(Math.toRadians(SwerveSubsystem.blModule.anglePos + robotPosition.getRotation().getDegrees())) * -Constants.bltMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference))
        / 4.0);
    deltaY = ((
         (Math.cos(Math.toRadians(SwerveSubsystem.frModule.anglePos + robotPosition.getRotation().getDegrees())) * Constants.frtMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference)
       + (Math.cos(Math.toRadians(SwerveSubsystem.flModule.anglePos + robotPosition.getRotation().getDegrees())) * -Constants.fltMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference)
       + (Math.cos(Math.toRadians(SwerveSubsystem.brModule.anglePos + robotPosition.getRotation().getDegrees())) * Constants.brtMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference)
       + (Math.cos(Math.toRadians(SwerveSubsystem.blModule.anglePos + robotPosition.getRotation().getDegrees())) * -Constants.bltMotor.getEncoder().getVelocity() * Constants.swerveDriveRatio * (1.0/3000.0) * Constants.swerveWheelCircumference))
        / 4.0);
    robotPosition  = new Pose2d(robotPosition.getX() + deltaX, robotPosition.getY() + deltaY, robotPosition.getRotation());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
