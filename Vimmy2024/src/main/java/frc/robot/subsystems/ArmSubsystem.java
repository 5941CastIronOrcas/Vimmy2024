package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
  public static RelativeEncoder armEncoder = Constants.armMotor1.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
  public static double armAngle = 0;
  public static double dist = 0;
  public static double g = Constants.gravity;
  public static boolean hasNote = false;
  public static double recalledValue;
  int noNoteFrames = 0;
  public ArmSubsystem() {}

  @Override
  public void periodic() {
    armAngle = -(Constants.armJointEncoder.get() * 360)+212.2;
    dist = PositionEstimator.distToSpeaker();

    recalledValue = ArduinoCommunication.RecallOneValue((byte) 0x2e);
    //if (!(recalledValue < 0)) {
      if(recalledValue < Constants.hasNoteTreshold) 
      {
        hasNote = true;
        noNoteFrames = 0;
      }
      else noNoteFrames++;
      if(noNoteFrames>40) hasNote = false;
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

  public static void moveArmTo(double a) {
    rotateArm(Functions.Clamp((Constants.armMotorPMult*(a - armAngle)) 
    -(Constants.armMotorDMult*armEncoder.getVelocity()), 
    -Constants.maxArmSpeed, Constants.maxArmSpeed));
     DriverDisplay.armTarget.setDouble(a);
  }
  
  public static void rotateArm(double t) {
    t = Functions.Clamp(t+(Constants.armMotorGravMult*Math.cos(Math.toRadians(armAngle))), -Functions.Clamp(0.2*(armAngle-Constants.minArmAngle), 0, 1), Functions.Clamp(-(0.2*(armAngle-Constants.maxArmAngle)), 0, 1));
    Constants.armMotor1.set((Constants.armMotor1Invert)?-t:t);
    Constants.armMotor2.set((Constants.armMotor2Invert)?-t:t);
    DriverDisplay.armThrottle.setDouble(t);
    

  }
  public static void SpinIntake(double input)
  {
    Constants.intakeMotor.set(-input);
  }
  public static void SpinShooter(double input)
  {
    Constants.lowerShooterMotor.set(input*Constants.bottomRpmMult);
    Constants.upperShooterMotor.set(input);
  }
  public static void SpinShooter(double lowIn, double upIn)
  {
    Constants.lowerShooterMotor.set(lowIn);
    Constants.upperShooterMotor.set(upIn);
  }
  public static void Intake(double input)
  {
    SpinIntake((hasNote)?0:input);
  }
  public static void IntakeRing() {
    moveArmTo(Constants.intakeAngle);
    Intake(0.5);
    SpinShooter(0);
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
  public static void DepositAmp() {
    SpinShooter(0.25);
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

  public static double GetSpeakerAngle() {
    double a = -4.87356607*Math.pow(10, 15);
    double b = 16.604623863;
    double c = -11.3487133839;
    double d = 43.1365385842;
    return a*Math.pow(dist + b, c)+d+2; //+5
    /*double s = Constants.launchSpeed + Functions.AltAxisCoord(PositionEstimator.velocity.x, PositionEstimator.velocity.y, SwerveSubsystem.angleToSpeaker);
    double d = Math.pow(s,4)-g*((g*dist*dist)+(2*Constants.speakerHeight*s*s));
    if (d>0) {
      return (-Math.toDegrees(Math.atan2((s*s-d),(g*dist)))+Constants.armAngleOffset);
    }
    return 0.0;*/
  }

  public static void PrepShooter(double speed) {
    SpinShooter(speed);
    moveArmTo(GetSpeakerAngle());
  }

  public static void ShootSpeaker() {
    boolean shooterFast = (Constants.upperShooterMotor.getEncoder().getVelocity() >= Constants.minShootRpm);
    boolean correctArmAngle = (Math.abs(GetSpeakerAngle()-armAngle) < Constants.armAngleVariation);
    if (shooterFast && correctArmAngle && PositionEstimator.atSpeakerAngle()) {
      SpinIntake(1);
    }
  }
  public static void ShootSpeaker2() {
    boolean shooterFast = (Constants.upperShooterMotor.getEncoder().getVelocity() >= Constants.minShootRpm);
    boolean correctArmAngle = (Math.abs(GetSpeakerAngle()-armAngle) < Constants.armAngleVariation);
    if (shooterFast && correctArmAngle && PositionEstimator.atSpeakerAngle()) {
      SpinIntake(1);
    }
    else
    {
      SpinIntake(0);
    }
  }

  public static void ShootTrap()
  {
    SpinShooter(Constants.trapShootSpeed);
    moveArmTo(Constants.trapShootAngle);
    boolean shooterFast = (Constants.lowerShooterMotor.getEncoder().getVelocity() >= Constants.trapMinRPM);
    boolean correctArmAngle = (Math.abs(Constants.trapShootAngle-armAngle) < Constants.armAngleVariation);
    if (shooterFast && correctArmAngle && SwerveSubsystem.atTargetPosition && SwerveSubsystem.atTargetAngle) {
      SpinIntake(1);
    }
  }


}