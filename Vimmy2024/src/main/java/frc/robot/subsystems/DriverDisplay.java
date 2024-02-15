// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DriverDisplay extends SubsystemBase {
  
  //NoteDetector
  public static ShuffleboardTab noteDetector = Shuffleboard.getTab("NoteDetector");
  public static GenericEntry showNote = noteDetector.add("HasNote 1",false).getEntry();
  public static GenericEntry showNotePitch = noteDetector.add("Note Pitch", 0).getEntry();
  public static GenericEntry showNoteYaw = noteDetector.add("Note Yaw", 0).getEntry();
 
  //Arm
  public static ShuffleboardTab arm = Shuffleboard.getTab("Arm");
  public static GenericEntry armAngle = arm.add("Arm Note", 0).getEntry();
  public static GenericEntry arduinoRecall = arm.add("Arduino Recall", 0).getEntry();
  public static GenericEntry armHasNote = arm.add("Has Note", false).getEntry();
  public static GenericEntry armTarget = arm.add("Arm Target", 0).getEntry();
  public static GenericEntry armThrottle = arm.add("Arm Throttle", 0).getEntry();

  //Swerve
  public static ShuffleboardTab swerve = Shuffleboard.getTab("Swerve");
  
  public static GenericEntry throttleMotorPower = swerve.add("Throttle motor power", 0).getEntry();
  public static GenericEntry throttleMotorRpm = swerve.add("Throttle motor rpm", 0).getEntry();
  public static GenericEntry targetSpeed = swerve.add("Target speed", 0).getEntry();
  public static GenericEntry targetVelocity = swerve.add("Target velocity", 0).getEntry();
  

  //Climber
  public static ShuffleboardTab climber = Shuffleboard.getTab("Climber");

  //GOA
  public static ShuffleboardTab goa = Shuffleboard.getTab("GOA");



  //Apriltags
  public static ShuffleboardTab apriltags = Shuffleboard.getTab("Apriltags");
    public static GenericEntry isPresent1 = apriltags.add("Is Present 1", false).getEntry();
    public static GenericEntry isPresent2 = apriltags.add("Is Present 2", false).getEntry();
    public static GenericEntry latency = apriltags.add("Latency", 0).getEntry();
    public static GenericEntry speed = apriltags.add("Speed", 0).getEntry();


  public DriverDisplay() {}


  @Override
  public void periodic() {
   
  

  }
}