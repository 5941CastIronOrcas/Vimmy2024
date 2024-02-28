// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PositionEstimator extends SubsystemBase {
  public static double robotYawDriverRelative = 0;
  private static double gyroYawOld = 0;
  public static double robotYawRate = 0;
  public static Pose2d robotPosition = new Pose2d();  
  public static Pose2d previousPosition = new Pose2d();
  public static Vector2D velocity = new Vector2D(0, 0);

  public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
 
  public static Transform3d robotToCam1 = new Transform3d(new Translation3d(0.0254, 0.2794, 0.4572), new Rotation3d(0,27,0));
  public static Transform3d robotToCam2 = new Transform3d(new Translation3d(0.0254, -0.2794, 0.4572), new Rotation3d(0,27,0));
  public static PhotonCamera camera1 = new PhotonCamera(Constants.apriltagCamera1Name);
  public static PhotonCamera camera2 = new PhotonCamera(Constants.apriltagCamera2Name);
  public static PhotonPipelineResult result1 = camera1.getLatestResult();
  public static PhotonPipelineResult result2 = camera2.getLatestResult();

  public static PhotonPoseEstimator photonPoseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1, robotToCam1);
  public static PhotonPoseEstimator photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1, robotToCam1);
  public static Vector2D[] deltaBuffer = new Vector2D[50];
  public static double latency1 = 0;
  public static double latency2 = 0;

  public static double sumX = 0;
  public static double sumY = 0;


  public static Boolean camCheck1() {
    //var result = camera1.getLatestResult();
    return result1.hasTargets();
  }

  public static Boolean camCheck2() {
    //var result2 = camera2.getLatestResult();
    return result2.hasTargets();
  }

  public static PhotonTrackedTarget obtainTargets() {
;
    //Sends back the most clear target and its data
    if (result1.hasTargets()) {
      return result1.getBestTarget();
    }
    else {
      return new PhotonTrackedTarget(0, 0, 0, 0, 0, null, null, 0, null, null);
    }

    //In Java/C++, You must always check if the result has a target via hasTargets()/HasTargets() 
    //before getting targets or else you may get a null pointer exception. Further, you must use 
    //the same result in every subsequent call in that loop.

  }

  public static Pose2d getEstimatedGlobalPose1() {
    //var emptyTarget = new PhotonTrackedTarget();
    if (!camCheck1()) {
      return previousPosition;
    }
    
    /*if (photonPoseEstimator.update().isPresent()) {
      return photonPoseEstimator.update().orElse(null).estimatedPose.toPose2d();
    } */
    
     
    try {
      return photonPoseEstimator1.update().get().estimatedPose.toPose2d();
    }
    catch(Exception e) {
      System.out.println("Caught Error: " + e);
    }
    
    return previousPosition;
  }
  public static Pose2d getEstimatedGlobalPose2() {
    //var emptyTarget = new PhotonTrackedTarget();
    if (!camCheck2()) {
      return previousPosition;
    }
    
    /*if (photonPoseEstimator.update().isPresent()) {
      return photonPoseEstimator.update().orElse(null).estimatedPose.toPose2d();
    } */
    
     
    try {
      return photonPoseEstimator2.update().get().estimatedPose.toPose2d();
    }
    catch(Exception e) {
      System.out.println("Caught Error: " + e);
    }
    
    return previousPosition;
  }

  /*public boolean isValid(Pose2d oldPose, Pose2d newPose) {
    double maxMovement = Constants.swerveMaxSpeed * camera1.getLatestResult().getLatencyMillis() * 0.001;
    double currentMovement = Functions.Pythagorean(oldPose.getX() - newPose.getX(), oldPose.getY() - newPose.getY());

    if (currentMovement > maxMovement) {
      return false;
    }
    else {
      return true;
    }
  }*/

  public static double distToSpeaker() {
    double supposed = Functions.Pythagorean((Robot.isRedAlliance?Constants.redSpeaker.x:Constants.blueSpeaker.x)-robotPosition.getX(), (Robot.isRedAlliance?Constants.redSpeaker.y:Constants.blueSpeaker.y)-robotPosition.getY());
    return supposed+0.523776;
  }
  public static double angleToSpeaker()
  {
    return 90-Math.toDegrees(Math.atan2((Robot.isRedAlliance?Constants.redSpeaker.y:Constants.blueSpeaker.y) - PositionEstimator.robotPosition.getY(),(Robot.isRedAlliance?Constants.redSpeaker.x:Constants.blueSpeaker.x) - PositionEstimator.robotPosition.getX()));
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
         (Math.sin(Math.toRadians(SwerveSubsystem.frModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.frModule.GetVelocity())
       + (Math.sin(Math.toRadians(SwerveSubsystem.flModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.flModule.GetVelocity())
       + (Math.sin(Math.toRadians(SwerveSubsystem.brModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.brModule.GetVelocity())
       + (Math.sin(Math.toRadians(SwerveSubsystem.blModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.blModule.GetVelocity()))
        / 4.0);
    velocity.y = ((
         (Math.cos(Math.toRadians(SwerveSubsystem.frModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.frModule.GetVelocity())
       + (Math.cos(Math.toRadians(SwerveSubsystem.flModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.flModule.GetVelocity())
       + (Math.cos(Math.toRadians(SwerveSubsystem.brModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.brModule.GetVelocity())
       + (Math.cos(Math.toRadians(SwerveSubsystem.blModule.GetAngle() + robotPosition.getRotation().getDegrees())) * SwerveSubsystem.blModule.GetVelocity()))
        / 4.0);
    result1 = camera1.getLatestResult();
    result2 = camera2.getLatestResult();
    latency1 = result1.getLatencyMillis();
    latency2 = result2.getLatencyMillis();

    for (int i = 49; i > 0; i--) {
      deltaBuffer[i] = deltaBuffer[i - 1];
    }
    deltaBuffer[0] = velocity;

    previousPosition = robotPosition;
    Pose2d globalPose1 = robotPosition;
    Pose2d globalPose2 = robotPosition;
    if(camCheck2())
    {
      globalPose2 = getEstimatedGlobalPose2();
      SmartDashboard.putNumber("X2", globalPose2.getX());
      SmartDashboard.putNumber("Y2", globalPose2.getY());
    }
    
    if (camCheck1()) {
      globalPose1 = getEstimatedGlobalPose1();
      if (robotPosition != globalPose1) {
        // apriltags present and information updated
        Pose2d combinedPoses = new Pose2d((globalPose1.getX() + globalPose2.getX())/2.0, (globalPose1.getY() + globalPose2.getY())/2.0, robotPosition.getRotation());
        robotPosition = new Pose2d(globalPose1.getX(), globalPose1.getY(), robotPosition.getRotation());
        /*for (int i = 0; i < (latency / 20); i++) {
          if (deltaBuffer[i] != null)
          {
            sumX += deltaBuffer[i].x;
            sumY += deltaBuffer[i].y;
          }
        }*/

        //robotPosition = new Pose2d(globalPose.getX() + sumX, globalPose.getY() + sumY, globalPose.getRotation());
      }
      else {
      // apriltags present, information not updated
      robotPosition = new Pose2d(robotPosition.getX() + velocity.x*0.02, robotPosition.getY() + velocity.y*0.02, robotPosition.getRotation());
      }
    }
    else {
      // no apriltags detected
      robotPosition  = new Pose2d(robotPosition.getX() + velocity.x*0.02, robotPosition.getY() + velocity.y*0.02, robotPosition.getRotation());
    }
    // if (camCheck1() && getEstimatedGlobalPose() != null) {
    //   robotPosition = getEstimatedGlobalPose();
    // }
//     double RawVisionX = -1;
//     double RawVisionY = -1;
//     try {
//       System.out.println("I'm trying I promise");
//       Optional<EstimatedRobotPose> erpo = photonPoseEstimator.update();
//       System.out.println("Found this shit");
//       if (!erpo.isEmpty()) {
//         EstimatedRobotPose erp = erpo.get();
//         System.out.println("Found erp");
//         Pose3d er = erp.estimatedPose;
//         System.out.println("Found er");
//         RawVisionX = er.getX();
//         System.out.println("got x");
//         RawVisionY = er.getY();
//         System.out.println("got y");
//       }
      
// //      RawVisionX = photonPoseEstimator.update().get().estimatedPose.getX(); 
// //      RawVisionY = photonPoseEstimator.update().get().estimatedPose.getY();
//     } catch (Exception e) {
//       System.out.println("exception found @ 197");
//       System.out.println(e);
//     }
//     SmartDashboard.putNumber("RawVisionX", RawVisionX);
//     SmartDashboard.putNumber("RawVisionY", RawVisionY);


    
  }

  // truespeed = deltaBuffer[camera1.getLatestResult().getLatencyMillis() / 20.0];

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
