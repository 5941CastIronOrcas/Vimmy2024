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
  public ArmSubsystem() {}

  @Override
  public void periodic() {
    armAngle = -(Constants.armJointEncoder.get() * 360)+211.8;
    dist = PositionEstimator.distToSpeaker();
    hasNote = false;    
    for (int i = 0; i < 10; i++) {
      recalledValue = ArduinoCommunication.RecallOneValue((byte) 0x2e);
      if (recalledValue < Constants.hasNoteTreshold) {
        hasNote = true;
        i = 10;
      }
    }
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
    +(Constants.armMotorGravMult*Math.cos(Math.toRadians(armAngle))) 
    -(Constants.armMotorDMult*armEncoder.getVelocity()), 
    -Constants.maxArmSpeed, Constants.maxArmSpeed));
     DriverDisplay.armTarget.setDouble(a);
  }
  
  public static void rotateArm(double t) {
    t = Functions.Clamp(t, -Functions.Clamp(0.25*(armAngle-Constants.minArmAngle), 0, 1), Functions.Clamp(-(0.25*(armAngle-Constants.maxArmAngle)), 0, 1));
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
    Constants.shooterMotor1.set(input);
    Constants.shooterMotor2.set(input);
  }
  public static void Intake(double input)
  {
    SpinIntake((hasNote)?0:input);
  }
  public static void IntakeRing() {
    moveArmTo(Constants.intakeAngle);
    Intake(1);
  }
  // public static void ShootAtAngle(double a) {
  //   SpinShooter(1);
  //   moveArmTo(a);
  //   if ((Constants.shooterMotor1.getEncoder().getVelocity() >= Constants.minShootRpm) && Math.abs(a-armAngle) < Constants.armAngleVariation) {
  //     SpinIntake(1);
  //   } else {
  //     SpinIntake(0);
  //   }
  // }
  public static void DepositAmp() {
    SpinShooter(0.1);
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
    return (-Math.toDegrees(Math.atan2(Constants.speakerHeight,dist))+Constants.armAngleOffset+90);
    /*double s = Constants.launchSpeed + Functions.AltAxisCoord(PositionEstimator.velocity.x, PositionEstimator.velocity.y, SwerveSubsystem.angleToSpeaker);
    double d = Math.pow(s,4)-g*((g*dist*dist)+(2*Constants.speakerHeight*s*s));
    if (d>0) {
      return (-Math.toDegrees(Math.atan2((s*s-d),(g*dist)))+Constants.armAngleOffset+90);
    }
    return 0.0;*/
  }

  public static void PrepShooter(double speed) {
    SpinShooter(speed);
    moveArmTo(GetSpeakerAngle());
  }

  public static void ShootSpeaker() {
    boolean shooterFast = (Constants.shooterMotor1.getEncoder().getVelocity() >= Constants.minShootRpm);
    boolean correctArmAngle = (Math.abs(GetSpeakerAngle()-armAngle) < Constants.armAngleVariation);
    boolean correctRobotAngle = (Math.abs(Math.atan2((Robot.isRedAlliance?Constants.redSpeaker.y:Constants.blueSpeaker.y) - PositionEstimator.robotPosition.getY(),(Robot.isRedAlliance?Constants.redSpeaker.x:Constants.blueSpeaker.x) - PositionEstimator.robotPosition.getX()) - PositionEstimator.robotPosition.getRotation().getRadians()) < Math.toRadians(Constants.swerveAngleVariation));
    if (shooterFast && correctArmAngle && correctRobotAngle) {
      SpinIntake(1);
    }
  }


}