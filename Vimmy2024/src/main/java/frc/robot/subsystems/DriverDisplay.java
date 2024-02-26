// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;
import frc.robot.utilityObjects.Vector2D;



public class DriverDisplay extends SubsystemBase {
  
  //NoteDetector
  public static ShuffleboardTab noteDetector = Shuffleboard.getTab("NoteDetector");
  public static GenericEntry showNote = noteDetector.add("Note Visible",false).getEntry();
  public static GenericEntry showNotePitch = noteDetector.add("Note Pitch", 0).getEntry();
  public static GenericEntry showNoteYaw = noteDetector.add("Note Yaw", 0).getEntry();
  public static GenericEntry noteDistance = noteDetector.add("Note Distance", 0).getEntry();
 
  //Arm
  public static ShuffleboardTab arm = Shuffleboard.getTab("Arm");
  public static GenericEntry armAngle = arm.add("Arm Angle", 0).getEntry();
  public static GenericEntry arduinoRecall = arm.add("Arduino Recall", 0).getEntry();
  public static GenericEntry armHasNote = arm.add("Has Note", false).getEntry();
  public static GenericEntry armTarget = arm.add("Arm Target", 0).getEntry();
  public static GenericEntry armThrottle = arm.add("Arm Throttle", 0).getEntry();
  public static GenericEntry motorPower1 = arm.add("ArmMotor1 Amps", 0).getEntry();
  public static GenericEntry motorPower2 = arm.add("ArmMotor2 Amps", 0).getEntry();
  public static GenericEntry motor1Rpm = arm.add("Shooter 1 Rpm", 0).getEntry();
  public static GenericEntry motor2Rpm = arm.add("Shooter 2 Rpm", 0).getEntry();
  public static GenericEntry range = arm.add("Range", 0).getEntry();


  //Swerve
  public static ShuffleboardTab swerve = Shuffleboard.getTab("Swerve");
  public static GenericEntry frAngle = swerve.add("Fr Angle", 0).getEntry();
  public static GenericEntry flAngle = swerve.add("Fl Angle", 0).getEntry();
  public static GenericEntry brAngle = swerve.add("Br Angle", 0).getEntry();
  public static GenericEntry blAngle = swerve.add("Bl Angle", 0).getEntry();
  public static GenericEntry frVelocity = swerve.add("Fr Velocity", 0).getEntry();
  public static GenericEntry flVelocity = swerve.add("Fl Velocity", 0).getEntry();
  public static GenericEntry brVelocity = swerve.add("Br Velocity", 0).getEntry();
  public static GenericEntry blVelocity = swerve.add("Bl Velocity", 0).getEntry();
  public static GenericEntry driveX = swerve.add("driveX", 0).getEntry();
  public static GenericEntry driveY = swerve.add("driveY", 0).getEntry();
  public static GenericEntry driveRotate = swerve.add("driveSpin", 0).getEntry();
  public static GenericEntry totalDriveAmps = swerve.add("Total Amps", 0).getEntry();
  public static GenericEntry atTargetPosition = swerve.add("atTargetPos", false).getEntry();
  public static GenericEntry atTargetAngle = swerve.add("atTargetAngle", false).getEntry();





  //Climber
  public static ShuffleboardTab climber = Shuffleboard.getTab("Climber");
  public static GenericEntry lClimberAngle = climber.add("LPosition", 0).getEntry();
  public static GenericEntry rClimberAngle = climber.add("RPosition", 0).getEntry(); 
  public static GenericEntry lClimberSwitch = climber.add("LSwitch", false).getEntry();
  public static GenericEntry rClimberSwitch = climber.add("RSwitch", false).getEntry();
  public static GenericEntry robotRoll = climber.add("RobotRoll", 0).getEntry();
  public static GenericEntry climberR = climber.add("ClimberR Input", 0).getEntry();
  public static GenericEntry climberL = climber.add("ClimberL Input", 0).getEntry();




  //GOA
  public static ShuffleboardTab goa = Shuffleboard.getTab("GOA");
  public static GenericEntry avoidanceX = goa.add("AvoidanceX", 0).getEntry();
  public static GenericEntry avoidanceY = goa.add("AvoidanceY", 0).getEntry();
  private final Field2d m_field = new Field2d();

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
    


  public DriverDisplay() {
  SmartDashboard.putData("Field", m_field); 

  }

  @Override
  public void periodic() {
    //Arm
    DriverDisplay.armAngle.setDouble(ArmSubsystem.armAngle);
    DriverDisplay.arduinoRecall.setDouble(ArmSubsystem.recalledValue);
    DriverDisplay.armHasNote.setBoolean(ArmSubsystem.hasNote);
    DriverDisplay.motorPower1.setDouble(Constants.armMotor1.getOutputCurrent());
    DriverDisplay.motorPower2.setDouble(Constants.armMotor2.getOutputCurrent());
    DriverDisplay.motor1Rpm.setDouble(Constants.shooterMotor1.getEncoder().getVelocity());
    DriverDisplay.motor2Rpm.setDouble(Constants.shooterMotor2.getEncoder().getVelocity());
    DriverDisplay.range.setDouble(PositionEstimator.distToSpeaker());
    



    //swerve
    DriverDisplay.frAngle.setDouble(SwerveSubsystem.frModule.GetAngle());
    DriverDisplay.flAngle.setDouble(SwerveSubsystem.flModule.GetAngle());
    DriverDisplay.brAngle.setDouble(SwerveSubsystem.brModule.GetAngle());
    DriverDisplay.blAngle.setDouble(SwerveSubsystem.blModule.GetAngle());
    DriverDisplay.frVelocity.setDouble(SwerveSubsystem.frModule.GetVelocity());
    DriverDisplay.flVelocity.setDouble(SwerveSubsystem.flModule.GetVelocity());
    DriverDisplay.brVelocity.setDouble(SwerveSubsystem.brModule.GetVelocity());
    DriverDisplay.blVelocity.setDouble(SwerveSubsystem.blModule.GetVelocity());
    DriverDisplay.totalDriveAmps.setDouble(Constants.fraMotor.getOutputCurrent() + Constants.flaMotor.getOutputCurrent() + Constants.braMotor.getOutputCurrent() + Constants.blaMotor.getOutputCurrent());
    DriverDisplay.atTargetPosition.setBoolean(SwerveSubsystem.atTargetPosition);
    DriverDisplay.atTargetAngle.setBoolean(SwerveSubsystem.atTargetAngle);

   
    

    //Note Detector
    DriverDisplay.showNoteYaw.setDouble(NoteDetector.noteYaw);
    DriverDisplay.showNotePitch.setDouble(NoteDetector.notePitch);
    DriverDisplay.showNote.setBoolean(NoteDetector.noteVisible);
    DriverDisplay.noteDistance.setDouble(NoteDetector.noteDist);


    //climber
    DriverDisplay.lClimberAngle.setDouble(ClimberSubsystem.lClimberAngle);
    DriverDisplay.rClimberAngle.setDouble(ClimberSubsystem.rClimberAngle);
    DriverDisplay.lClimberSwitch.setBoolean(Constants.lClimberSwitch.get());
    DriverDisplay.rClimberSwitch.setBoolean(Constants.rClimberSwitch.get());
    DriverDisplay.robotRoll.setDouble(Constants.gyro.getRoll().getValueAsDouble());
    DriverDisplay.climberR.setDouble(Constants.climberMotorR.get());
    DriverDisplay.climberL.setDouble(Constants.climberMotorL.get());


    //goa
    /*Vector2D nono = GOAGuidanceSystem.GetAvoidanceVector();
    DriverDisplay.avoidanceX.setDouble(nono.x);
    DriverDisplay.avoidanceY.setDouble(nono.y);
    Pose2d positionPose2d = new Pose2d(PositionEstimator.robotPosition.getX(), PositionEstimator.robotPosition.getY(), new Rotation2d(-PositionEstimator.robotPosition.getRotation().getRadians() - Math.PI));
    for (int i = 0; i < GOADataSynthesizer.allObstacles.length; i++)
    {
      if(GOADataSynthesizer.allObstacles[i] != null)
      {
        m_field.getObject(Integer.toString(i)).setPose(new Pose2d(GOADataSynthesizer.allObstacles[i].x, GOADataSynthesizer.allObstacles[i].y, new Rotation2d(0)));
      }
    }
    m_field.setRobotPose(positionPose2d);*/  //GOA disabled for now
    
    
    

    




    //position estimator
    DriverDisplay.isPresent1.setBoolean(PositionEstimator.camCheck1());
    DriverDisplay.isPresent2.setBoolean(PositionEstimator.camCheck2());
    DriverDisplay.latency.setDouble(PositionEstimator.camera1.getLatestResult().getLatencyMillis());
    DriverDisplay.speed.setDouble(Functions.Pythagorean(PositionEstimator.velocity.x, PositionEstimator.velocity.y));
    DriverDisplay.robotX.setDouble(PositionEstimator.robotPosition.getX());
    DriverDisplay.robotY.setDouble(PositionEstimator.robotPosition.getY());
    DriverDisplay.driverYaw.setDouble(PositionEstimator.robotYawDriverRelative);
    DriverDisplay.fieldYaw.setDouble( Functions.DeltaAngleDeg(0, PositionEstimator.robotPosition.getRotation().getDegrees()));
    

   
    
  

  }
}