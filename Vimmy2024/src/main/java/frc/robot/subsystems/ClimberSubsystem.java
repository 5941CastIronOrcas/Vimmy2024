// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static void moveClimbers(double in, double difference) {
    Constants.climberMotorL.set((in+difference/2.0)*(Constants.climberMotorLInvert?-1:1));
    Constants.climberMotorR.set((in-difference/2.0)*(Constants.climberMotorRInvert?-1:1));
  }

  public static void autoBalance(double speed) {
    moveClimbers(speed,Functions.Clamp((Constants.climberBalancePMult*(-Constants.gyro.getRoll().getValueAsDouble())), -Constants.climberMaxSpeed, Constants.climberMaxSpeed));
  }
}
