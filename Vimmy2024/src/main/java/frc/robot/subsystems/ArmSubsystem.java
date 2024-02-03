package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;

public class ArmSubsystem extends SubsystemBase {
  public static RelativeEncoder armEncoder = Constants.armMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
  public static double armAngle;
  public ArmSubsystem() {}

  @Override
  public void periodic() {
    armAngle = armEncoder.getPosition() * 360;
    SmartDashboard.putNumber("Arm Angle", armAngle);
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
    Constants.armMotor.set((Constants.armMotorInvert)?-t:t);
    SmartDashboard.putNumber("Arm Throttle", (Constants.armMotorInvert)?-t:t);
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
    boolean pressed = false;
    for (int i = 0; i < 8 ; i++)
    {
      if (Constants.limitSwitches[i].get())
      {
        pressed = true;
      }
    }
    SpinIntake((pressed)?0:input);
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