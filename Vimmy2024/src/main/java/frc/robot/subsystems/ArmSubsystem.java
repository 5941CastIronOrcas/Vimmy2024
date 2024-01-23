// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void rotateArm(double t1, double t2) {
    Constants.armMotor1.set((Constants.armMotor1Invert)?-t1:t1);
    Constants.armMotor2.set((Constants.armMotor2Invert)?-t2:t2);
    SmartDashboard.putNumber("Arm Motor 1 Throttle", (Constants.armMotor1Invert)?-t1:t1);
    SmartDashboard.putNumber("Arm Motor 2 Throttle", (Constants.armMotor2Invert)?-t2:t2);
  }
}
