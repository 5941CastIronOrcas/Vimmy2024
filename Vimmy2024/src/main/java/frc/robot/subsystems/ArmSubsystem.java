// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;

public class ArmSubsystem extends SubsystemBase {
  
  public double armAngle; //current raise angle of the arm, in degrees. Horizontal is 0 degrees, down is negative, up is positive. 
  
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {}

  @Override
  public void periodic() {
    armAngle = Constants.armMotor1.getEncoder().getPosition() * 360 * Constants.armGearRatio; //sets current raise angle to encoder positions.
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void moveArmTo(double a1, double a2) {
    rotateArm(Functions.Clamp((Constants.armMotorPMult * (a1 - armAngle))
    +(Constants.armMotorGravMult * Math.cos(Math.toRadians(armAngle)))
    +(Constants.armMotorDMult*Constants.armMotor1.getEncoder().getVelocity()), 
    -Constants.maxArmSpeed, Constants.maxArmSpeed));
    SmartDashboard.putNumber("Arm Motor Target", a1);
  }
  
  public void rotateArm(double t1) {
    Constants.armMotor1.set((Constants.armMotor1Invert)?-t1:t1);
    SmartDashboard.putNumber("Arm Motor Throttle", (Constants.armMotor1Invert)?-t1:t1);
  }
}
