// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;

public class ClimberSubsystem extends SubsystemBase {
  public static double lClimberAngle;
  public static double rClimberAngle;
  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {
    Constants.climberMotorL.getEncoder().setPosition(Constants.climberMaxHeight);
    Constants.climberMotorR.getEncoder().setPosition(Constants.climberMaxHeight);
  }

  @Override
  public void periodic() {
    if (!Constants.lClimberSwitch.get()) Constants.climberMotorL.getEncoder().setPosition(0.0);
    if (!Constants.rClimberSwitch.get()) Constants.climberMotorR.getEncoder().setPosition(0.0);
    lClimberAngle = Constants.climberMotorL.getEncoder().getPosition();
    rClimberAngle = Constants.climberMotorR.getEncoder().getPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static void moveClimbers(double in, double difference) {
    Constants.climberMotorL.set(Functions.Clamp(in+(difference/2.0)*(Constants.climberMotorLInvert?-1:1), (!Constants.lClimberSwitch.get()?0:-1) , lClimberAngle>=Constants.climberMaxHeight?0:1));
    Constants.climberMotorR.set(Functions.Clamp(in-(difference/2.0)*(Constants.climberMotorRInvert?-1:1), (!Constants.rClimberSwitch.get()?0:-1) , rClimberAngle>=Constants.climberMaxHeight?0:1));
  }

  public static void autoBalance(double speed) {
    moveClimbers(speed,Functions.Clamp((Constants.climberBalancePMult*(-Constants.gyro.getRoll().getValueAsDouble())), 
    -Constants.climberMaxSpeed, Constants.climberMaxSpeed));
  }
}