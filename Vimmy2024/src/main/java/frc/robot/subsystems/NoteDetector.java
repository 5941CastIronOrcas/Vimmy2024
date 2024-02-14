// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NoteDetector extends SubsystemBase {

  public static PhotonCamera camera = new PhotonCamera(Constants.noteDetectionCameraName);
  public static boolean noteVisible = false;
  public static double notePitch = 0;
  public static double noteYaw = 0;
  

  public NoteDetector() {}

  public static Boolean camCheck() {
    var result = camera.getLatestResult();

    return result.hasTargets();
  }
  
 
  public static PhotonTrackedTarget obtainTargets() {
    var result = camera.getLatestResult();
      //Sends back the most clear target and its data
      return result.getBestTarget();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
      notePitch = camera.getLatestResult().getBestTarget().getPitch() + ArmSubsystem.armAngle;
      noteYaw = camera.getLatestResult().getBestTarget().getYaw();
    }

   
    DriverDisplay.showNoteYaw.setDouble(noteYaw);
   DriverDisplay.showNotePitch.setDouble(notePitch);
   DriverDisplay.showNote.setBoolean(noteVisible);
    
  }

  
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
