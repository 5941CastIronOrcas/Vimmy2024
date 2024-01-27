// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Main;
import frc.robot.Robot;

//import frc.robot.subsystems.SwerveModule;
import java.lang.Math;

/** The SwerveSubsystem class is used to calculate the angle and throttle of each swerve module, taking the sticks as input.
 **/

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static double flThrottleOut = 0;
  public static double frThrottleOut = 0;
  public static double blThrottleOut = 0;
  public static double brThrottleOut = 0;
  public static SwerveModule flModule = new SwerveModule(Constants.flaMotor,Constants.fltMotor,Constants.flEncoder,true,false,-45);
  public static SwerveModule frModule = new SwerveModule(Constants.fraMotor,Constants.frtMotor,Constants.frEncoder,true,true,45);
  public static SwerveModule blModule = new SwerveModule(Constants.blaMotor,Constants.bltMotor,Constants.blEncoder,true,false,45);
  public static SwerveModule brModule = new SwerveModule(Constants.braMotor,Constants.brtMotor,Constants.brEncoder,true,true,-45);
  public SwerveSubsystem() {}

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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  public static void DriveTo(double x, double y, double angle, double speedLimit, double turnLimit)
  {
    double angleToTarget = Math.atan2(x-PositionEstimator.robotPosition.getX(), y-PositionEstimator.robotPosition.getY());
    double pComponent = Constants.swerveDriveToPMult*Functions.Pythagorean(x-PositionEstimator.robotPosition.getX(), y-PositionEstimator.robotPosition.getY());
    double dComponent = Constants.swerveDriveToDMult*Functions.Pythagorean(PositionEstimator.deltaX, PositionEstimator.deltaY);
    double output = Functions.Clamp(pComponent - dComponent, 0, speedLimit);
    double xComponent = Functions.DeadZone(output * Math.sin(angleToTarget), Constants.swerveDriveToDeadZone);
    double yComponent = Functions.DeadZone(output * Math.cos(angleToTarget), Constants.swerveDriveToDeadZone);
    DriveFieldOrientedAtAngle(xComponent, yComponent, angle, turnLimit);
  }
  public static void DriveFieldOriented(double x, double y, double turn)
  {
    Drive(x*Math.cos(Math.toRadians(-PositionEstimator.robotPosition.getRotation().getDegrees()))+y*Math.sin(Math.toRadians(-PositionEstimator.robotPosition.getRotation().getDegrees())), y*Math.cos(Math.toRadians(-PositionEstimator.robotPosition.getRotation().getDegrees()))+x*Math.sin(Math.toRadians(PositionEstimator.robotPosition.getRotation().getDegrees())), turn);
  }
  
  public static void DriveFieldOrientedAtAngle(double x, double y, double angle, double turnLimit)
  {
    /*DriveDriverOriented(DriverStation.getAlliance() == DriverStation.Alliance.Red?y:-y, 
    DriverStation.getAlliance() == DriverStation.Alliance.Red?-x:x, 
    Functions.Clamp(-Constants.swerveAutoTurnPMult*Functions.DeltaAngleDegrees(angle, robotYawFieldRelative), 
    -Constants.swerveAutoTurnMaxSpeed*Functions.Clamp(turnLimit, 0, 1), 
    Constants.swerveAutoTurnMaxSpeed*Functions.Clamp(turnLimit, 0, 1)));*/
    DriveDriverOrientedAtAngle(Robot.isRedAlliance?y:-y, 
    Robot.isRedAlliance?-x:x, 
    Functions.FieldToDriverAngle(angle), turnLimit);
  }

  public static void DriveDriverOriented(double LSX, double LSY, double RSX)
  {
    Drive(LSX*Math.cos(Math.toRadians(-PositionEstimator.robotYawDriverRelative))+LSY*Math.sin(Math.toRadians(-PositionEstimator.robotYawDriverRelative)), LSY*Math.cos(Math.toRadians(-PositionEstimator.robotYawDriverRelative))+LSX*Math.sin(Math.toRadians(PositionEstimator.robotYawDriverRelative)), RSX);
  }

  public static void DriveDriverOrientedAtAngle(double LSX, double LSY, double angle, double turnLimit)
  {
    turnLimit = Functions.Clamp(turnLimit, 0, 1);
    
    DriveDriverOriented(LSX, LSY, 
    Functions.Clamp(-Constants.swerveAutoTurnPMult*Functions.DeadZone(
      Functions.DeltaAngleDeg(angle, PositionEstimator.robotYawDriverRelative), 
      Constants.swerveAutoTurnDeadZone), 
      -Constants.swerveAutoTurnMaxSpeed*turnLimit, 
      Constants.swerveAutoTurnMaxSpeed*turnLimit));
  }
  
  public static void Drive(double x, double y, double rotate) {
    SmartDashboard.putNumber("turn", rotate);
    double flx =  x - (Constants.turnMult * rotate);
    double fly =  y - (Constants.turnMult * rotate);
    double frx =  x - (Constants.turnMult * rotate);
    double fry =  y + (Constants.turnMult * rotate);
    double blx =  x + (Constants.turnMult * rotate);
    double bly =  y - (Constants.turnMult * rotate);
    double brx =  x + (Constants.turnMult * rotate);
    double bry =  y + (Constants.turnMult * rotate);
    double maxDist = Functions.Max(new double[]{
      Math.sqrt(Math.pow(flx,2) + Math.pow(fly,2)),
      Math.sqrt(Math.pow(frx,2) + Math.pow(fry,2)),
      Math.sqrt(Math.pow(blx,2) + Math.pow(bly,2)),
      Math.sqrt(Math.pow(brx,2) + Math.pow(bry,2)),1.0}); //use x*x not Math.pow()
    double flAngle = -Math.toDegrees(Math.atan2(fly,flx))-90;
    double frAngle = -Math.toDegrees(Math.atan2(fry,frx))-90;
    double blAngle = -Math.toDegrees(Math.atan2(bly,blx))-90;
    double brAngle = -Math.toDegrees(Math.atan2(bry,brx))-90;
    double flThrottle = Math.sqrt(Math.pow(flx,2) + Math.pow(fly,2)) / maxDist;
    double frThrottle = Math.sqrt(Math.pow(frx,2) + Math.pow(fry,2)) / maxDist;
    double blThrottle = Math.sqrt(Math.pow(blx,2) + Math.pow(bly,2)) / maxDist;
    double brThrottle = Math.sqrt(Math.pow(brx,2) + Math.pow(bry,2)) / maxDist; //use x*x not Math.pow()
    flThrottleOut += Functions.Clamp(flThrottle-flThrottleOut, -Constants.maxThrottleChange, Constants.maxThrottleChange);
    frThrottleOut += Functions.Clamp(frThrottle-frThrottleOut, -Constants.maxThrottleChange, Constants.maxThrottleChange);
    blThrottleOut += Functions.Clamp(blThrottle-blThrottleOut, -Constants.maxThrottleChange, Constants.maxThrottleChange);
    brThrottleOut += Functions.Clamp(brThrottle-brThrottleOut, -Constants.maxThrottleChange, Constants.maxThrottleChange);
    flModule.Drive(flAngle, flThrottle);
    frModule.Drive(frAngle, frThrottle);
    blModule.Drive(blAngle, blThrottle);
    brModule.Drive(brAngle, brThrottle);
    SmartDashboard.putNumber("FRAT", frAngle);
    SmartDashboard.putNumber("FLAT", flAngle);
    SmartDashboard.putNumber("BRAT", brAngle);
    SmartDashboard.putNumber("BLAT", blAngle);

  }

  
}
