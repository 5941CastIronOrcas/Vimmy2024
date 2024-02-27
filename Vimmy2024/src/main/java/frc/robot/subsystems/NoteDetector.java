// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NoteDetector extends SubsystemBase {

  public static PhotonCamera camera = new PhotonCamera(Constants.noteDetectionCameraName);
  public static boolean noteVisible = false;
  public static double notePitch = 0;
  public static double noteYaw = 0;
  public static double noteDist = 0;
  public static PhotonPipelineResult result = camera.getLatestResult();
  public static PhotonTrackedTarget target = result.getBestTarget();
  

  public NoteDetector() {}

  public static Boolean camCheck() {
    return result.hasTargets();
  }
  
 
  public static PhotonTrackedTarget obtainTarget() {
      return result.getBestTarget();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
    target = obtainTarget();
    if(camCheck() && ArmSubsystem.armAngle < 40)
    {
      noteVisible = true;
    }
    else
    {
      noteVisible = false;
    }
  
    if(noteVisible)
    {
      try
      {
        notePitch = target.getPitch() + ArmSubsystem.armAngle;
        noteYaw = target.getYaw();
      }
      catch (Exception e)
      {

      }
    }
    else
    {
      notePitch = 0;
      noteYaw = 0;
    }

    noteDist = -Constants.cameraHeight / Math.tan(Math.toRadians(notePitch + Constants.cameraAngle));
  }

  
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
