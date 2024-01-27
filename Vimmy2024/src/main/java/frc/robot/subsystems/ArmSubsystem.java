package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;

public class ArmSubsystem extends SubsystemBase {
    public double armAngle;
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {}

  @Override
  public void periodic() {
    armAngle = Constants.armMotor1.getEncoder().getPosition() * 360 * Constants.armGearRatio;
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void moveArmTo(double a1, double a2) {
    rotateArm(Functions.Clamp((Constants.armMotorPMult * (a1 - armAngle)) 
    +(Constants.armMotorGravMult) * Math.cos(Math.toRadians(armAngle)) 
    +(Constants.armMotorDMult*Constants.armMotor1.getEncoder().getVelocity()), 
    -Constants.maxArmSpeed, Constants.maxArmSpeed));
    SmartDashboard.putNumber("Arm Motor Target", a1);
  }
  public void rotateArm(double t1) {
    Constants.armMotor1.set((Constants.armMotor1Invert)?-t1:t1);
    SmartDashboard.putNumber("Arm Motor Throttle", (Constants.armMotor1Invert)?-t1:t1);
  }
  public void Spinintake(double input)
  {
    Constants.intakeMotor.set(input);
  }
  public void SpinShooter(double input)
  {
    Constants.shooterMotor1.set(input);
    Constants.shooterMotor2.set(-input);
  }
}