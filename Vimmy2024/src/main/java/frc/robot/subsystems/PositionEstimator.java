// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;
import frc.robot.utilityObjects.Vector2D;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PositionEstimator extends SubsystemBase {
  public static double robotYawDriverRelative = 0;
  private static double gyroYawOld = 0;
  public static double robotYawRate = 0;
  public static Pose2d robotPosition = new Pose2d();  
  public static Pose2d previousPosition = new Pose2d();
  public static Vector2D velocity = new Vector2D(0, 0);

  public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
 
  public static Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
  public static PhotonCamera camera1 = new PhotonCamera(Constants.apriltagCamera1Name);

  public static PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1, robotToCam);
  public static Vector2D[] deltaBuffer = new Vector2D[50];


  public static Boolean camCheck() {
    var result = camera1.getLatestResult();

    return result.hasTargets();
  }

  public static PhotonTrackedTarget obtainTargets() {
    var result = camera1.getLatestResult();

    if  (result.hasTargets()) {
      //Sends back the most clear target and its data
      return result.getBestTarget();
    } 
    else {
      return new PhotonTrackedTarget();
    }
  }

  public static Pose2d getEstimatedGlobalPose() {
    //var emptyTarget = new PhotonTrackedTarget();
    if (!camCheck()) {
      return previousPosition;
    }
    
    /*if (photonPoseEstimator.update().isPresent()) {
      return photonPoseEstimator.update().orElse(null).estimatedPose.toPose2d();
    }*/
    
    
    try{
      if (photonPoseEstimator.update().isPresent()) {
        return photonPoseEstimator.update().get().estimatedPose.toPose2d();
      }
    }
    catch(Exception e)
    {
      System.out.println("Caught Error: " + e);
    }
    
    return previousPosition;
  }

  public boolean isValid(Pose2d oldPose, Pose2d newPose)
  {
    double maxMovement = Constants.swerveMaxSpeed * camera1.getLatestResult().getLatencyMillis() * 0.001;
    double currentMovement = Functions.Pythagorean(oldPose.getX() - newPose.getX(), oldPose.getY() - newPose.getY());
    if (currentMovement > maxMovement) {
      return false;
    }
    else {
      return true;
    }
  }

  public static double distToSpeaker() {
    return Functions.Pythagorean((Robot.isRedAlliance?Constants.redSpeaker.x:Constants.blueSpeaker.x)-robotPosition.getX(), (Robot.isRedAlliance?Constants.redSpeaker.y:Constants.blueSpeaker.y)-robotPosition.getY());
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    robotYawDriverRelative = Functions.DeltaAngleDeg(0, -Constants.gyro.getYaw().getValueAsDouble());
    //robotYawRate = Constants.gyro.getRate();
    robotYawRate = -(Constants.gyro.getYaw().getValueAsDouble() - gyroYawOld)/0.02;
    gyroYawOld = Constants.gyro.getYaw().getValueAsDouble();

    if(Robot.isRedAlliance) {
      robotPosition = new Pose2d(robotPosition.getX(), robotPosition.getY(), new Rotation2d(Math.toRadians(Functions.DeltaAngleDeg(0, robotYawDriverRelative - 90))));
    } else if (Robot.isBlueAlliance) {
            robotPosition = new Pose2d(robotPosition.getX(), robotPosition.getY(), new Rotation2d(Math.toRadians(Functions.DeltaAngleDeg(0, robotYawDriverRelative + 90))));      
    } else {
      robotPosition = new Pose2d(robotPosition.getX(), robotPosition.getY(), new Rotation2d(Math.toRadians(Functions.DeltaAngleDeg(0, robotYawDriverRelative))));
    }
     velocity.x = ((
         (Math.sin(Math.toRadians(SwerveSubsystem.frModule.anglePos + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.frModule.velocity)
       + (Math.sin(Math.toRadians(SwerveSubsystem.flModule.anglePos + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.flModule.velocity)
       + (Math.sin(Math.toRadians(SwerveSubsystem.brModule.anglePos + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.brModule.velocity)
       + (Math.sin(Math.toRadians(SwerveSubsystem.blModule.anglePos + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.blModule.velocity))
        / 4.0);
    velocity.y = ((
         (Math.cos(Math.toRadians(SwerveSubsystem.frModule.anglePos + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.frModule.velocity)
       + (Math.cos(Math.toRadians(SwerveSubsystem.flModule.anglePos + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.flModule.velocity)
       + (Math.cos(Math.toRadians(SwerveSubsystem.brModule.anglePos + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.brModule.velocity)
       + (Math.cos(Math.toRadians(SwerveSubsystem.blModule.anglePos + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.blModule.velocity))
        / 4.0);

    for (int i = 49; i < 0; i--) {
      deltaBuffer[i] = deltaBuffer[i - 1];
    }
    deltaBuffer[49] = velocity;

    previousPosition = robotPosition;
    Pose2d globalPose = robotPosition;

    if (camCheck()) {
      if (getEstimatedGlobalPose() != null) {
        globalPose = getEstimatedGlobalPose();
      }
    }
    if (camCheck()) {
      if (robotPosition != globalPose && isValid(robotPosition, globalPose)) {
        // apriltags present and information updated
        robotPosition = globalPose;
      }
      else {
      // apriltags present, information not updated
      robotPosition = new Pose2d(robotPosition.getX() + velocity.x, robotPosition.getY() + velocity.y, robotPosition.getRotation());
      }
    }
    else {
      // no apriltags detected
      robotPosition  = new Pose2d(robotPosition.getX() + velocity.x, robotPosition.getY() + velocity.y, robotPosition.getRotation());
    }
    if (camCheck() && getEstimatedGlobalPose() != null) {
      robotPosition = getEstimatedGlobalPose();
    }
    SmartDashboard.putBoolean("isPresent", camCheck());
    SmartDashboard.putNumber("Latency", camera1.getLatestResult().getLatencyMillis());
    SmartDashboard.putNumber("Speed in m/s", 50*Functions.Pythagorean(velocity.x, velocity.y));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
