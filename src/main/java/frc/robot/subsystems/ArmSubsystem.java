package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;


public class ArmSubsystem extends SubsystemBase {
  public static RelativeEncoder armEncoder = Constants.armMotor1.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192); //the encoder that reads the arm's position
  public static double armAngle = 0; //the current angle of the arm
  public static double oldSpeakerAngle = 0; //the angle at which the robot should put it's arm in order to shoot into the speaker one frame ago
  public static double newSpeakerAngle = 0; //the angle at which the robot should put it's arm in order to shoot into the speaker
  public static double dist = 0; //the distance between the center of the robot and the speaker
  public static boolean inRange = false; //if the robot is within the minimum shooting range
  public static double g = Constants.gravity; //gravitational acceleration m/s/s
  public static boolean hasNote = false; //whether or not the robot is currently holding a note
  public static boolean shooterFast = false; //whether or not the shooter is spinning at or above the minimum shoot speed
  public static boolean correctArmAngle = false; //whether or not the arm is close enough to the correct angle to shoot to get the note in the speaker
  public static double recalledValue; //the distance measured by the ultrasonic hooked into the arduino
  int noNoteFrames = 0; //the number of frames that have passed since the last time the ultrasonic sensor saw a Note
  public ArmSubsystem() {}

  @Override
  public void periodic() {
    armAngle = -(Constants.armJointEncoder.get() * 360)+212.2; //sets the ArmAngle appropriately
    oldSpeakerAngle = newSpeakerAngle; 
    newSpeakerAngle = GetSpeakerAngle();
    dist = PositionEstimator.distToSpeaker();
    inRange = dist < Constants.maxShootingRange; //checks if robot is in range of the speaker

    recalledValue = Arduino.getCallArduino((byte) 0x2e); //????????
    //if (!(recalledValue < 0)) {
      if(recalledValue < Constants.hasNoteTreshold) //if the ultrasonic can see a Note
      {
        hasNote = true; //immediately assume the note is real
        noNoteFrames = 0; //set the number of frames since a note was seen to 0
      }
      else noNoteFrames++; //if the ultrasonic can't see a note, increase noNoteFrames
      if(noNoteFrames>40) hasNote = false; //if it has been 40 frames since the ultrasonic saw a Note, then assume the robot is not holding a Note
    //}

    //this code is not working
    /* 
    hasNote = false;    
    for (int i = 0; i < 10; i++) {
      recalledValue = ArduinoCommunication.RecallOneValue((byte) 0x2e);
      if (recalledValue < Constants.hasNoteTreshold) {
        hasNote = true;
        i = 10;
      }
    }
    */
    // this is an old version of code for limit switches 
    /*
    for (int i = 0; i < Constants.noteDetectionSwitches.length ; i++) {
      if (!Constants.noteDetectionSwitches[i].get())
      {
        hasNote = true;
      }
    }
    */
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static void moveArmTo(double a) { //uses a pd controller to go to a given angle.
    a = Functions.Clamp(a, Constants.minArmAngle, Constants.maxArmAngle);
    rotateArm(Functions.Clamp((Constants.armMotorPMult*(a - armAngle)) 
    -(Constants.armMotorDMult*armEncoder.getVelocity()), 
    -Constants.maxArmSpeed, Constants.maxArmSpeed));
     DriverDisplay.armTarget.setDouble(a);
  }

  public static void manualMoveArmTo() { //moves the arm to the angle it would need to be at if the robot was right up against the speaker.
    moveArmTo(16.1);
  }
  
  public static void rotateArm(double t) { //moves the arm with a certain amount of power, ranging from 1 to -1. the funky stuff in the first line just limits the arm angle.
    t = Functions.Clamp(t, -Functions.Clamp(0.2*(armAngle-Constants.minArmAngle), 0, 1), Functions.Clamp(-(0.2*(armAngle-Constants.maxArmAngle)), 0, 1)) + (Constants.armMotorGravMult*Math.cos(Math.toRadians(armAngle)));
    Constants.armMotor1.set((Constants.armMotor1Invert)?-t:t);
    Constants.armMotor2.set((Constants.armMotor2Invert)?-t:t);
    DriverDisplay.armThrottle.setDouble(t);
  }
  public static void SpinIntake(double input) //spins the intake at the inputted speed (-1 to 1), applying safety limits as needed.
  {
    Constants.intakeMotor.set(Functions.Clamp(-input, -1, armAngle < 15 ? 0 : 1));
  }
  public static void SpinShooter(double input) //spins the shooter at the inputted speed (-1 to 1), applies multiplier to equalize the shooter rpms.
  {
    input *= 0.5;
    Constants.lowerShooterMotor.set(input*Constants.bottomRpmMult);
    Constants.upperShooterMotor.set(input);
  }
  public static void SpinShooter(double lowIn, double upIn) //spins the shooter at the two inputted speeds (-1 to 1); allowing for differential in wheel speeds.
  {
    Constants.lowerShooterMotor.set(lowIn*Constants.bottomRpmMult);
    Constants.upperShooterMotor.set(upIn);
  }
  public static void Intake(double input) //spins the intake motor in an attempt to pick up a Note, stops once a Note has been collected.  
  {
    Constants.intakeMotor.setIdleMode(IdleMode.kBrake);
    SpinIntake((hasNote)?0:input);
  }
  public static void IntakeRing() { //moves the arm to the intake position, and tries to pick up a Note
    moveArmTo(Constants.intakeAngle);
    Intake(0.5);
  }
  // public static void ShootAtAngle(double a) {
  //   SpinShooter(1);
  //   moveArmTo(a);
  //   if ((Constants.lowerShooterMotor.getEncoder().getVelocity() >= Constants.minShootRpm) && Math.abs(a-armAngle) < Constants.armAngleVariation) {
  //     SpinIntake(1);
  //   } else {
  //     SpinIntake(0);
  //   }
  // }
  public static void DepositAmp() { //moves the arm to the amp despositing position, and preps the shooter wheels to eject the Note
    SpinShooter(1,1);
    moveArmTo(Constants.ampDepositAngle);
    /*if (Math.abs(Constants.ampDepositAngle-armAngle) < Constants.armAngleVariation) 
    {
      SpinIntake(1);
    }
    else 
    {
      SpinIntake(0);
    }*/
  }
  public static void DepositAmpSafety() { //same as DepositAmp, but tries to avoid hitting the arm against the field wall
    SpinShooter(0.1,0.7);
    if (SwerveSubsystem.atTargetPosition) moveArmTo(Constants.ampDepositAngle);
    else moveArmTo(85);
  }

  public static double GetSpeakerAngle() { //returns the angle that the arm should be at in order to shoot into the speaker
    double a = -70.8237335782; //-3.366287681*Math.pow(10, 15);
    double b = -0.349465145747;
    double c = -0.262092184395;
    double d = 87.2383991914;
    return a*Math.pow(dist + b, c)+d; //+5
    /*double s = Constants.launchSpeed + Functions.AltAxisCoord(PositionEstimator.velocity.x, PositionEstimator.velocity.y, SwerveSubsystem.angleToSpeaker);
    double d = Math.pow(s,4)-g*((g*dist*dist)+(2*Constants.speakerHeight*s*s));
    if (d>0) {
      return (-Math.toDegrees(Math.atan2((s*s-d),(g*dist)))+Constants.armAngleOffset);
    }
    return 0.0;*/
  }
  public static double GetPredictedSpeakerAngle() { //this takes the angle to speakers's rate of change and multiplies it by the time it would take for the note to reach the speaker. this gives the predicted angle the robot should be at while moving.
    double aSpeed = (newSpeakerAngle - oldSpeakerAngle) * 50;
    return GetSpeakerAngle() + (aSpeed * (Functions.Pythagorean(PositionEstimator.distToSpeaker(), Constants.speakerHeight) / Constants.launchSpeed));
  }

  public static void PrepShooter(double speed) { //gets the shooter up to the right speed and moves the arm to the correct angle.
    SpinShooter(speed); 
    moveArmTo(GetSpeakerAngle());
  }

  public static void ShootSpeaker() { //if everything is ready, it will spin the intake motors, pushing the note into the shooter wheels.
    Constants.intakeMotor.setIdleMode(IdleMode.kCoast);
    shooterFast = (Constants.upperShooterMotor.getEncoder().getVelocity() >= Constants.minShootRpm) || (Constants.upperShooterMotor.getEncoder().getVelocity() > Constants.minShootRpm/2 && PositionEstimator.distToSpeaker() < 1);
    correctArmAngle = (Math.abs(GetSpeakerAngle()-armAngle) < Constants.armAngleVariation);
    if (shooterFast && correctArmAngle && PositionEstimator.atSpeakerAngle() && inRange) {
      SpinIntake(1);
    }
  }
  public static void ShootSpeaker2() { //same as regular shoot speaker, but will cancel if the requirements aren't met.
    Constants.intakeMotor.setIdleMode(IdleMode.kCoast);
    shooterFast = (Constants.upperShooterMotor.getEncoder().getVelocity() >= Constants.minShootRpm) || (Constants.upperShooterMotor.getEncoder().getVelocity() > Constants.minShootRpm/2 && PositionEstimator.distToSpeaker() < 1);
    correctArmAngle = (Math.abs(GetSpeakerAngle()-armAngle) < Constants.armAngleVariation);
    if (shooterFast && correctArmAngle && PositionEstimator.atSpeakerAngle() && inRange) {
      SpinIntake(1);
    }
    else
    {
      SpinIntake(0);
    }
  }

  public static void ShootTrap() //spins shooter and moves arm to the correct angle. then, it spins the intake to shoot the ring.
  {
    SpinShooter(Constants.trapShootSpeed);
    moveArmTo(Constants.trapShootAngle);
    boolean trapShooterFast = (Constants.lowerShooterMotor.getEncoder().getVelocity() >= Constants.trapMinRPM);
    boolean trapCorrectArmAngle = (Math.abs(Constants.trapShootAngle-armAngle) < Constants.armAngleVariation);
    if (trapShooterFast && trapCorrectArmAngle && SwerveSubsystem.atTargetPosition && SwerveSubsystem.atTargetAngle) {
      SpinIntake(1);
    }
  }


}