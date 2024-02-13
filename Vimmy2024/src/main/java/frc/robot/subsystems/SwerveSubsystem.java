// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;

import java.lang.Math;

/** The SwerveSubsystem class is used to calculate the angle and throttle of each swerve module, taking the sticks as input.
 **/

public class SwerveSubsystem extends SubsystemBase {
  public static double xOut = 0;
  public static double yOut = 0;
  public static double flThrottleOut = 0;
  public static double frThrottleOut = 0;
  public static double blThrottleOut = 0;
  public static double brThrottleOut = 0;
  public static SwerveModule flModule = new SwerveModule(Constants.flaMotor,Constants.fltMotor,Constants.flEncoder,true,Constants.fltInvert,45);
  public static SwerveModule frModule = new SwerveModule(Constants.fraMotor,Constants.frtMotor,Constants.frEncoder,true,Constants.frtInvert,-45);
  public static SwerveModule blModule = new SwerveModule(Constants.blaMotor,Constants.bltMotor,Constants.blEncoder,true,Constants.bltInvert,-45);
  public static SwerveModule brModule = new SwerveModule(Constants.braMotor,Constants.brtMotor,Constants.brEncoder,true,Constants.frtInvert,45);
  public SwerveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  public static void DriveTo(double x, double y, double angle, double speedLimit, double turnLimit, double XOffset, double YOffset)
  {
    /*double angleToTarget = 90-Math.atan2(y-PositionEstimator.robotPosition.getY(), x-PositionEstimator.robotPosition.getX());
    double pComponent = Constants.swerveDriveToPMult*Functions.Pythagorean(x-PositionEstimator.robotPosition.getX(), y-PositionEstimator.robotPosition.getY());
    double dComponent = Constants.swerveDriveToDMult*Functions.Pythagorean(PositionEstimator.deltaX, PositionEstimator.deltaY);
    double output = Functions.Clamp(pComponent - dComponent, 0, speedLimit);
    double xComponent = Functions.DeadZone(output * Math.sin(angleToTarget), Constants.swerveDriveToDeadZone);
    double yComponent = Functions.DeadZone(output * Math.cos(angleToTarget), Constants.swerveDriveToDeadZone);
    DriveFieldOrientedAtAngle(xComponent+(Robot.isRedAlliance?-YOffset:YOffset), yComponent+(Robot.isRedAlliance?XOffset:-XOffset), angle, turnLimit);*/
    double angleToTarget = Math.atan2(x-PositionEstimator.robotPosition.getX(), y-PositionEstimator.robotPosition.getY());
    double pComponent = Constants.swerveDriveToPMult*Functions.DeadZone(Functions.Pythagorean(x-PositionEstimator.robotPosition.getX(), y-PositionEstimator.robotPosition.getY()), Constants.swerveDriveToDeadZone);
    double dComponent = Constants.swerveDriveToDMult*Functions.Pythagorean(PositionEstimator.velocity.x, PositionEstimator.velocity.y);
    double output = Functions.Clamp(pComponent - dComponent, 0, speedLimit);
    double xComponent = output * Math.sin(angleToTarget);
    double yComponent = output * Math.cos(angleToTarget);
    DriveFieldOrientedAtAngle(xComponent+(Robot.isRedAlliance?-YOffset:YOffset), yComponent+(Robot.isRedAlliance?XOffset:-XOffset), angle, turnLimit);
  }
  public static void DriveFieldOriented(double x, double y, double turn)
  {
    DriveDriverOriented(Robot.isRedAlliance?y:-y, Robot.isRedAlliance?-x:x, turn);
  }
  
  public static void DriveFieldOrientedAtAngle(double x, double y, double angle, double turnLimit)
  {
    DriveDriverOrientedAtAngle(Robot.isRedAlliance?y:-y, Robot.isRedAlliance?-x:x, Functions.FieldToDriverAngle(angle), turnLimit);
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
      Constants.swerveAutoTurnDeadZone)-Constants.swerveAutoTurnDMult*PositionEstimator.robotYawRate, 
      -Constants.swerveAutoTurnMaxSpeed*turnLimit, 
      Constants.swerveAutoTurnMaxSpeed*turnLimit));
  }

  public static void FaceSpeaker(double x, double y, double turnLimit) {
    DriveFieldOrientedAtAngle(x, y, Math.atan2((Robot.isRedAlliance?Constants.redSpeaker.y:Constants.blueSpeaker.y) - PositionEstimator.robotPosition.getY(),(Robot.isRedAlliance?Constants.redSpeaker.x:Constants.blueSpeaker.x) - PositionEstimator.robotPosition.getX()), turnLimit);
  }

  public static void CollectNote(double XOffset, double YOffset, double speedLimit) {
    double a = NoteDetector.noteVisible?(PositionEstimator.robotYawDriverRelative + NoteDetector.noteYaw):PositionEstimator.robotYawDriverRelative;
    DriveDriverOrientedAtAngle(Math.cos(-a+90), Math.sin(-a+90), a, speedLimit);
  }
  
  public static void Drive(double x, double y, double rotate) {
    double currentMaxAccel = Constants.swerveMaxAccel;
    //uncomment the below line to enable adaptive acceleration limiter
    //currentMaxAccel = Constants.swerveMaxAccelExtended + (Math.cos(ArmSubsystem.armAngle)*(Constants.swerveMaxAccel-Constants.swerveMaxAccelExtended));
    xOut += Functions.Clamp(x-xOut, -currentMaxAccel, currentMaxAccel);
    yOut += Functions.Clamp(y-yOut, -currentMaxAccel, currentMaxAccel);
    double flx =  xOut + (Constants.turnMult * rotate);
    double fly =  yOut + (Constants.turnMult * rotate);
    double frx =  xOut + (Constants.turnMult * rotate);
    double fry =  yOut - (Constants.turnMult * rotate);
    double blx =  xOut - (Constants.turnMult * rotate);
    double bly =  yOut + (Constants.turnMult * rotate);
    double brx =  xOut - (Constants.turnMult * rotate);
    double bry =  yOut - (Constants.turnMult * rotate);
    double maxDist = Functions.Max(new double[]{
      Functions.Pythagorean(flx, fly),
      Functions.Pythagorean(frx, fry),
      Functions.Pythagorean(blx, bly),
      Functions.Pythagorean(brx, bry),1.0}); //use x*x not Math.pow()
    double flAngle = 90-Math.toDegrees(Math.atan2(fly,flx));
    double frAngle = 90-Math.toDegrees(Math.atan2(fry,frx));
    double blAngle = 90-Math.toDegrees(Math.atan2(bly,blx));
    double brAngle = 90-Math.toDegrees(Math.atan2(bry,brx));
    double flThrottle = Functions.Pythagorean(flx, fly) / maxDist;
    double frThrottle = Functions.Pythagorean(frx, fry) / maxDist;
    double blThrottle = Functions.Pythagorean(blx, bly) / maxDist;
    double brThrottle = Functions.Pythagorean(brx, bry) / maxDist; //use x*x not Math.pow()
    flThrottleOut += Functions.Clamp(flThrottle-flThrottleOut, -Constants.maxThrottleChange, Constants.maxThrottleChange);
    frThrottleOut += Functions.Clamp(frThrottle-frThrottleOut, -Constants.maxThrottleChange, Constants.maxThrottleChange);
    blThrottleOut += Functions.Clamp(blThrottle-blThrottleOut, -Constants.maxThrottleChange, Constants.maxThrottleChange);
    brThrottleOut += Functions.Clamp(brThrottle-brThrottleOut, -Constants.maxThrottleChange, Constants.maxThrottleChange);
    flModule.Drive(flAngle, flThrottleOut);
    frModule.Drive(frAngle, frThrottleOut);
    blModule.Drive(blAngle, blThrottleOut);
    brModule.Drive(brAngle, brThrottleOut);
  }
  
}
