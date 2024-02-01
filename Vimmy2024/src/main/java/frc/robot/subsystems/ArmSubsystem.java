package frc.robot.subsystems;

import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;

public class ArmSubsystem extends SubsystemBase {
  public RelativeEncoder armEncoder = Constants.armMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
  public double armAngle;
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {}

  @Override
  public void periodic() {
    armAngle = armEncoder.getPosition() * 360;
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void moveArmTo(double a) {
    rotateArm(Functions.Clamp((Constants.armMotorPMult*(a - armAngle)) 
    +(Constants.armMotorGravMult*Math.cos(Math.toRadians(armAngle))) 
    +(Constants.armMotorDMult*armEncoder.getVelocity()), 
    -Constants.maxArmSpeed, Constants.maxArmSpeed));
    SmartDashboard.putNumber("Arm Motor Target", a);
  }
  public void rotateArm(double t) {
    Constants.armMotor.set((Constants.armMotorInvert)?-t:t);
    SmartDashboard.putNumber("Arm Motor Throttle", (Constants.armMotorInvert)?-t:t);
  }
  public void SpinIntake(double input)
  {
    Constants.intakeMotor.set(input);
  }
  public void SpinShooter(double input)
  {
    Constants.shooterMotor1.set(input);
    Constants.shooterMotor2.set(-input);
  }
  public void Intake(double input)
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

}