// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;
import frc.robot.utilityObjects.Vector2D;

import java.lang.Math;

/** The SwerveSubsystem class is used to calculate the angle and throttle of each swerve module, taking the sticks as input.
 **/

public class SwerveSubsystem extends SubsystemBase {
  public static double xOut = 0.0; //x and y inputs with clamped acceleration
  public static double yOut = 0.0;
  public static double flThrottleOut = 0.0; //same as xOut and yOut but for throttle for each swerve module
  public static double frThrottleOut = 0.0;
  public static double blThrottleOut = 0.0;
  public static double brThrottleOut = 0.0;
  public static double oldAngle = 0.0; //used to find the rate of change of the angle to the speaker
  public static double newAngle = 0.0;
  public static Vector2D goaAvoidVector = new Vector2D(0, 0); //vector used to avoid obstacles
  public static double goaAvoidAngle = 0.0; //used to point intake away from obstacles
  public static SwerveModule flModule = new SwerveModule(Constants.flaMotor,Constants.fltMotor,Constants.flEncoder,true,Constants.fltInvert,45); //assigns each swerve module
  public static SwerveModule frModule = new SwerveModule(Constants.fraMotor,Constants.frtMotor,Constants.frEncoder,true,Constants.frtInvert,-45);
  public static SwerveModule blModule = new SwerveModule(Constants.blaMotor,Constants.bltMotor,Constants.blEncoder,true,Constants.bltInvert,-45);
  public static SwerveModule brModule = new SwerveModule(Constants.braMotor,Constants.brtMotor,Constants.brEncoder,true,Constants.frtInvert,45);
  public static boolean atTargetPosition = false;
  public static boolean atTargetAngle = false;
  public SwerveSubsystem() {}

  @Override
  public void periodic() {
    oldAngle = newAngle;
    newAngle = PositionEstimator.angleToSpeaker();
    goaAvoidVector = GOAGuidanceSystem.GetAvoidanceVector();
    goaAvoidAngle = Math.toDegrees(Math.atan2(goaAvoidVector.x, goaAvoidVector.y)) + 180;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  public static void DriveTo(double x, double y, double angle, double speedLimit, double turnLimit, double XOffset, double YOffset) //uses a PD controller and the Drive function to drive to a given point on the field.
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
    atTargetPosition = pComponent < 0.001;
    double dComponent = Constants.swerveDriveToDMult*Functions.Pythagorean(PositionEstimator.velocity.x, PositionEstimator.velocity.y);
    double output = Functions.Clamp(pComponent - dComponent, 0, speedLimit);
    double xComponent = output * Math.sin(angleToTarget);
    double yComponent = output * Math.cos(angleToTarget);
    DriveFieldOrientedAtAngle(xComponent+(Robot.isRedAlliance?-YOffset:YOffset), yComponent+(Robot.isRedAlliance?XOffset:-XOffset), angle, turnLimit);
  }
  public static void GOADriveTo(double x, double y, double angle, double speedLimit, double turnLimit, double XOffset, double YOffset) //same as DriveTo, but avoids obstacles with GOA.
  {
    Vector2D v = GOAGuidanceSystem.GetDriveVector(x, y, goaAvoidVector, speedLimit);
    DriveFieldOrientedAtAngle(v.x, v.y, angle, turnLimit);
  }
  public static void GOADriveToProtectIntake (double x, double y, double speedLimit, double turnLimit, double XOffset, double YOffset) { //same as GOADriveTo, but rotates to protect the intake.
    GOADriveTo(x, y, goaAvoidAngle, speedLimit, turnLimit, XOffset, YOffset);
  }
  
  public static void DriveFieldOriented(double x, double y, double turn) //same as DriveDriverOriented, but drives relative to the field instead of the driver.
  {
    DriveDriverOriented(Robot.isRedAlliance?y:-y, Robot.isRedAlliance?-x:x, turn);
  }
  
  public static void DriveFieldOrientedAtAngle(double x, double y, double angle, double turnLimit) //same as DriveDriverOrientedAtAngle, but drives relative to the field instead of the driver.
  {
    DriveDriverOrientedAtAngle(Robot.isRedAlliance?y:-y, Robot.isRedAlliance?-x:x, Functions.FieldToDriverAngle(angle), turnLimit);
  }

  public static void DriveDriverOriented(double LSX, double LSY, double RSX) //similar to Drive, but when the driver pushes up on the left stick, the robot moves forward from their point of view instead of the robot's.
  {
    Drive(LSX*Math.cos(Math.toRadians(-PositionEstimator.robotYawDriverRelative))+LSY*Math.sin(Math.toRadians(-PositionEstimator.robotYawDriverRelative)), LSY*Math.cos(Math.toRadians(-PositionEstimator.robotYawDriverRelative))+LSX*Math.sin(Math.toRadians(PositionEstimator.robotYawDriverRelative)), RSX);
  }

  public static void DriveDriverOrientedAtAngle(double LSX, double LSY, double angle, double turnLimit) //uses a PD controller to rotate the robot to a specific angle, instead of rotating the robot at a certain speed. otherwise, the same as DriveDriverOriented.
  {
    turnLimit = Functions.Clamp(turnLimit, 0, 1);
    double t = Functions.Clamp(-Constants.swerveAutoTurnPMult*Functions.DeadZone(
      Functions.DeltaAngleDeg(angle, PositionEstimator.robotYawDriverRelative), 
      Constants.swerveAutoTurnDeadZone)-Constants.swerveAutoTurnDMult*PositionEstimator.robotYawRate, 
      -Constants.swerveAutoTurnMaxSpeed*turnLimit, 
      Constants.swerveAutoTurnMaxSpeed*turnLimit);
    atTargetAngle = Math.abs(t) < 0.001;

    DriveDriverOriented(LSX, LSY, t);
  }
  public static void DriveDriverOrientedProtectIntake(double LSX, double LSY, double turnLimit) { //same as DriveDriverOriented, but rotates to protect the intake.
    DriveDriverOrientedAtAngle(LSX, LSY, Functions.FieldToDriverAngle(goaAvoidAngle), turnLimit);
  }
  public static void DriveToProtectIntake(double x, double y, double speedLimit, double turnLimit, double XOffset, double YOffset) { //same as DriveTo, but rotates to protect the intake.
    DriveTo(x, y, goaAvoidAngle, speedLimit, turnLimit, XOffset, YOffset);
  }

  public static void FaceSpeaker(double x, double y, double turnLimit) { //rotates the robot to face the speaker.
    //double aSpeed = (newAngle - oldAngle) * 50;
    //double a = PositionEstimator.angleToSpeaker() + (aSpeed * (PositionEstimator.distToSpeaker() / Constants.launchSpeed));
    //DriveDriverOrientedAtAngle(x, y, Functions.FieldToDriverAngle(a), turnLimit);

    double aSpeed = 0;
    if(Functions.Pythagorean(PositionEstimator.velocity.x, PositionEstimator.velocity.y) > 0.1)
    {
      aSpeed = (newAngle - oldAngle) * 50;
    }
    double a = PositionEstimator.angleToSpeaker() + (aSpeed * 0.5);
    DriveDriverOrientedAtAngle(x, y, Functions.FieldToDriverAngle(a), turnLimit);
  }

  public static void SquareDriveTo(double x, double y, double angle, double speedLimit, double turnLimit, double XOffset, double YOffset, boolean xFirst) { //same as DriveTo, but moves on one axis before moving on the other.
    if (xFirst) {
      DriveTo(x, Math.abs(y-PositionEstimator.robotPosition.getY())<Constants.swerveSquareDriveToDeadZone?y:PositionEstimator.robotPosition.getY(), angle, speedLimit, turnLimit, XOffset, YOffset);
    }
    else {
      DriveTo(Math.abs(x-PositionEstimator.robotPosition.getX())<Constants.swerveSquareDriveToDeadZone?x:PositionEstimator.robotPosition.getX(), y, angle, speedLimit, turnLimit, XOffset, YOffset);
    }
  }

  public static void GoToAmp(double speedLimit, double turnLimit, double XOffset, double YOffset) //drives to the amp.
  {
    Vector2D target = Robot.isRedAlliance?Constants.redAmpDepositPosition:Constants.blueAmpDepositPosition;
    DriveTo(target.x, PositionEstimator.robotPosition.getY(), 0, speedLimit, turnLimit, XOffset, YOffset);
  }

  public static void CollectNote(double XOffset, double YOffset, double speedLimit) { //if the note is visible, it will drive toward it, and intake the note.
    /*if (NoteDetector.noteVisible && !ArmSubsystem.hasNote) {
      double a = PositionEstimator.robotYawDriverRelative + NoteDetector.noteYaw;
      double x = -Math.cos(Math.toRadians(-a+90)) * NoteDetector.noteDist + XOffset;
      double y = -Math.sin(Math.toRadians(-a+90)) * NoteDetector.noteDist + YOffset;
      DriveDriverOrientedAtAngle(Functions.Clamp(x * Constants.swerveCollectNotePMult, -speedLimit, speedLimit), 
        Functions.Clamp(y * Constants.swerveCollectNotePMult, -speedLimit, speedLimit), 
        a, speedLimit);
    }*/
    speedLimit = Functions.Clamp(speedLimit, 0, 1);
    double a = PositionEstimator.robotYawDriverRelative + NoteDetector.noteYaw;
    double x = 0;
    double y = 0;
    if(NoteDetector.noteVisible && !ArmSubsystem.hasNote && Math.abs(Functions.DeltaAngleDeg(a, PositionEstimator.robotYawDriverRelative)) < 15 && ArmSubsystem.armAngle < 2)
    {
      double goSpeed = Constants.swerveCollectNotePMult * NoteDetector.noteDist;
      x = goSpeed * -Math.cos(Math.toRadians(-a+90));
      y = goSpeed * -Math.sin(Math.toRadians(-a+90));
    }
    DriveDriverOrientedAtAngle(Functions.Clamp(XOffset+x, -speedLimit, speedLimit), Functions.Clamp(YOffset+y, -speedLimit, speedLimit), a, speedLimit);

  }
  
  public static void Drive(double x, double y, double rotate) { //this is the basis of the swerve code
    double currentMaxAccel = Constants.swerveMaxAccel;
    //uncomment the below line to enable adaptive acceleration limiter
    currentMaxAccel = Functions.Clamp(Constants.swerveMaxAccelExtended + (Math.cos(Math.toRadians(ArmSubsystem.armAngle))*(Constants.swerveMaxAccel-Constants.swerveMaxAccelExtended)), 0.25, 2.0);
    xOut += Functions.Clamp(x-xOut, -currentMaxAccel, currentMaxAccel); //xOut and yOut are x and y, but the acceleration is limited.
    yOut += Functions.Clamp(y-yOut, -currentMaxAccel, currentMaxAccel);
    double flx =  xOut + (Constants.turnMult * rotate); //the x and y coordinates of each wheel. since the rotation affects each wheel differently, rotation is either added or subtracted from x and y.
    double fly =  yOut + (Constants.turnMult * rotate);
    double frx =  xOut + (Constants.turnMult * rotate);
    double fry =  yOut - (Constants.turnMult * rotate);
    double blx =  xOut - (Constants.turnMult * rotate);
    double bly =  yOut + (Constants.turnMult * rotate);
    double brx =  xOut - (Constants.turnMult * rotate);
    double bry =  yOut - (Constants.turnMult * rotate);
    double maxDist = Functions.Max(new double[]{ //the max amplitude of the vectors made by the x and y coordinates. the minimum value is 1.
      Functions.Pythagorean(flx, fly),
      Functions.Pythagorean(frx, fry),
      Functions.Pythagorean(blx, bly),
      Functions.Pythagorean(brx, bry),1.0});
    double flAngle = 90-Math.toDegrees(Math.atan2(fly,flx)); //the angle each module turns to.
    double frAngle = 90-Math.toDegrees(Math.atan2(fry,frx));
    double blAngle = 90-Math.toDegrees(Math.atan2(bly,blx));
    double brAngle = 90-Math.toDegrees(Math.atan2(bry,brx));
    double flThrottle = Functions.Pythagorean(flx, fly) / maxDist; //the power assigned to each throttle motor. each value is divided by the max, as to keep all values at or under 1.
    double frThrottle = Functions.Pythagorean(frx, fry) / maxDist;
    double blThrottle = Functions.Pythagorean(blx, bly) / maxDist;
    double brThrottle = Functions.Pythagorean(brx, bry) / maxDist;
    flThrottleOut += Functions.Clamp(flThrottle-flThrottleOut, -Constants.maxThrottleChange, Constants.maxThrottleChange); //same as xOut and yOut, but limiting the throttle instead.
    frThrottleOut += Functions.Clamp(frThrottle-frThrottleOut, -Constants.maxThrottleChange, Constants.maxThrottleChange);
    blThrottleOut += Functions.Clamp(blThrottle-blThrottleOut, -Constants.maxThrottleChange, Constants.maxThrottleChange);
    brThrottleOut += Functions.Clamp(brThrottle-brThrottleOut, -Constants.maxThrottleChange, Constants.maxThrottleChange);
    flModule.Drive(flAngle, flThrottleOut); //telling each individual module to move.
    frModule.Drive(frAngle, frThrottleOut);
    blModule.Drive(blAngle, blThrottleOut);
    brModule.Drive(brAngle, brThrottleOut);
    DriverDisplay.driveX.setDouble(x);
    DriverDisplay.driveY.setDouble(y);
    DriverDisplay.driveRotate.setDouble(rotate);
  }
  
}
