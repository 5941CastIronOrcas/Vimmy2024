package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;

public class ArmSubsystem extends SubsystemBase {
  public static RelativeEncoder armEncoder = Constants.armMotor1.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
  public static double armAngle = 0;
  public static boolean hasNote = false;
  public ArmSubsystem() {}

  @Override
  public void periodic() {
    armAngle = armEncoder.getPosition() * 360;
    SmartDashboard.putNumber("Arm Angle", armAngle);
    SmartDashboard.putBoolean("Has Note", hasNote);
    hasNote = false;
    for (int i = 0; i < Constants.noteDetectionSwitches.length ; i++)
    {
      if (!Constants.noteDetectionSwitches[i].get())
      {
        hasNote = true;
      }
    }
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static void moveArmTo(double a) {
    rotateArm(Functions.Clamp((Constants.armMotorPMult*(a - armAngle)) 
    +(Constants.armMotorGravMult*Math.cos(Math.toRadians(armAngle))) 
    +(Constants.armMotorDMult*armEncoder.getVelocity()), 
    -Constants.maxArmSpeed, Constants.maxArmSpeed));
    SmartDashboard.putNumber("Arm Target", a);
  }
  public static void rotateArm(double t) {
    Constants.armMotor1.set((Constants.armMotor1Invert)?-t:t);
    Constants.armMotor2.set((Constants.armMotor2Invert)?-t:t);
    SmartDashboard.putNumber("Arm Throttle", t);
  }
  public static void SpinIntake(double input)
  {
    Constants.intakeMotor.set(input);
  }
  public static void SpinShooter(double input)
  {
    Constants.shooterMotor1.set(input);
    Constants.shooterMotor2.set(-input);
  }
  public static void Intake(double input)
  {
    SpinIntake((hasNote)?0:input);
  }
  public static void IntakeRing() {
    moveArmTo(Constants.intakeAngle);
    Intake(1);
  }
  public static void ShootAtAngle(double a) {
    SpinShooter(1);
    moveArmTo(a);
    if ((Constants.intakeMotor.getEncoder().getVelocity() >= Constants.minShootRpm) && Math.abs(a-armAngle) < Constants.armAngleVariation) {
      SpinIntake(1);
    } else {
      SpinIntake(0);
    }
  }
  public static void DepositAmp() {
    SpinShooter(0.2);
    moveArmTo(Constants.ampDepositAngle);
    if (Math.abs(Constants.ampDepositAngle-armAngle) < Constants.armAngleVariation) 
    {
      SpinIntake(1);
    }
    else 
    {
      SpinIntake(0);
    }
    
  }
}