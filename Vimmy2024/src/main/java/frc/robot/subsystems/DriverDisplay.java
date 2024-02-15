// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;
import frc.robot.utilityObjects.Vector2D;



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
  public static GenericEntry frAngle = swerve.add("Fr Module", 0).getEntry();
  public static GenericEntry flAngle = swerve.add("Fl Module", 0).getEntry();
  public static GenericEntry brAngle = swerve.add("Br module", 0).getEntry();
  public static GenericEntry blAngle = swerve.add("Bl Module", 0).getEntry();
  

  
  

  //Climber
  public static ShuffleboardTab climber = Shuffleboard.getTab("Climber");

  //GOA
  public static ShuffleboardTab goa = Shuffleboard.getTab("GOA");
  public static GenericEntry avoidanceX = goa.add("AvoidanceX", 0).getEntry();
  public static GenericEntry avoidanceY = goa.add("AvoidanceY", 0).getEntry();


  //position estimator
  public static ShuffleboardTab position = Shuffleboard.getTab("Position Estimator");
    public static GenericEntry isPresent1 = position.add("Is Present 1", false).getEntry();
    public static GenericEntry isPresent2 = position.add("Is Present 2", false).getEntry();
    public static GenericEntry speed = position.add("speed", 0).getEntry();
    public static GenericEntry latency = position.add("Latency", 0).getEntry();
    public static GenericEntry robotX = position.add("Robot X", 0).getEntry();
    public static GenericEntry robotY = position.add("Robot Y", 0).getEntry();
    public static GenericEntry driverYaw = position.add("DriverYaw", 0).getEntry();
    public static GenericEntry fieldYaw = position.add("FieldYaw", 0).getEntry();
    public static GenericEntry robotRoll = position.add("RobotRoll", 0).getEntry();
    public static GenericEntry altAxis = position.add("AltAxisCoord Test", 0).getEntry();


  public DriverDisplay() {}


  @Override
  public void periodic() {
    //Arm
    DriverDisplay.armAngle.setDouble(ArmSubsystem.armAngle);
    DriverDisplay.arduinoRecall.setDouble(ArduinoCommunication.RecallOneValue((byte) 0x2e));
    DriverDisplay.armHasNote.setBoolean(ArmSubsystem.hasNote);

    //swerve
    DriverDisplay.frAngle.setDouble(SwerveSubsystem.frModule.anglePos);
    DriverDisplay.flAngle.setDouble(SwerveSubsystem.flModule.anglePos);
    DriverDisplay.brAngle.setDouble(SwerveSubsystem.brModule.anglePos);
    DriverDisplay.blAngle.setDouble(SwerveSubsystem.blModule.anglePos);


    //Note Detector
    DriverDisplay.showNoteYaw.setDouble(NoteDetector.noteYaw);
    DriverDisplay.showNotePitch.setDouble(NoteDetector.notePitch);
    DriverDisplay.showNote.setBoolean(NoteDetector.noteVisible);





    //goa
    Vector2D nono = GOAGuidanceSystem.GetAvoidanceVector();
    DriverDisplay.avoidanceX.setDouble(nono.x);
    DriverDisplay.avoidanceY.setDouble(nono.y);
  

    //position estimator
    DriverDisplay.isPresent1.setBoolean(PositionEstimator.camCheck());
    DriverDisplay.isPresent2.setBoolean(PositionEstimator.camCheck2());
    DriverDisplay.latency.setDouble(PositionEstimator.camera1.getLatestResult().getLatencyMillis());
    DriverDisplay.speed.setDouble(Functions.Pythagorean(PositionEstimator.velocity.x, PositionEstimator.velocity.y));
    DriverDisplay.robotX.setDouble(PositionEstimator.robotPosition.getX());
    DriverDisplay.robotY.setDouble(PositionEstimator.robotPosition.getY());
    DriverDisplay.driverYaw.setDouble(PositionEstimator.robotYawDriverRelative);
    DriverDisplay.fieldYaw.setDouble( Functions.DeltaAngleDeg(0, PositionEstimator.robotPosition.getRotation().getDegrees()));
    DriverDisplay.robotRoll.setDouble(Constants.gyro.getRoll().getValueAsDouble());
    DriverDisplay.altAxis.setDouble(Functions.AltAxisCoord(3, 2, 0.25*Math.PI));

   
    
  

  }
}