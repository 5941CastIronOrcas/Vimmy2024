// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DriverDisplay extends SubsystemBase {

  public static ShuffleboardTab noteDetector = Shuffleboard.getTab("NoteDetector");
  
  private GenericEntry showNote = noteDetector.add("HasNote 1",false).getEntry();
  private GenericEntry showNotePitch = noteDetector.add("Note Pitch", 0).getEntry();
  private GenericEntry showNoteYaw = noteDetector.add("Note Yaw", 0).getEntry();
 


  public DriverDisplay() {}


  @Override
  public void periodic() {
   showNoteYaw.setDouble(NoteDetector.noteYaw);
   showNotePitch.setDouble(NoteDetector.notePitch);
   showNote.setBoolean(NoteDetector.noteVisible);
    

  }
}