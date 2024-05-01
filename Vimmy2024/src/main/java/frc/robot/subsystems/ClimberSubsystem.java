// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;

public class ClimberSubsystem extends SubsystemBase {
  public static double lClimberAngle;
  public static double rClimberAngle;
  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() { //initializes the climbers
    Constants.climberMotorL.getEncoder().setPosition(0);
    Constants.climberMotorR.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    if (!Constants.lClimberSwitch.get()) Constants.climberMotorL.getEncoder().setPosition(0.0); //sets climber position to 0 if it's at the bottom
    if (!Constants.rClimberSwitch.get()) Constants.climberMotorR.getEncoder().setPosition(0.0);
    lClimberAngle = Constants.climberMotorL.getEncoder().getPosition();
    rClimberAngle = Constants.climberMotorR.getEncoder().getPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static void moveClimbers(double in, double difference) { //moves the climbers, adding or subtracting half of difference to make the difference between the two climber powers equal to that number.
    Constants.climberMotorL.set(Functions.Clamp(in+(difference/2.0)*(Constants.climberMotorLInvert?-1:1), 
    !Constants.lClimberSwitch.get()?0: Functions.Clamp(Constants.climberReductionMult*(lClimberAngle-Constants.climberSmoothingEnd)-Constants.climberMaxHitSpeed, -1, -Constants.climberMaxHitSpeed), 
    lClimberAngle>=Constants.climberMaxHeight?0:1));

    Constants.climberMotorR.set(Functions.Clamp(in-(difference/2.0)*(Constants.climberMotorRInvert?-1:1), 
    !Constants.rClimberSwitch.get()?0:Functions.Clamp(Constants.climberReductionMult*(lClimberAngle-Constants.climberSmoothingEnd)-Constants.climberMaxHitSpeed, -1, -Constants.climberMaxHitSpeed), 
    rClimberAngle>=Constants.climberMaxHeight?0:1));
  }

  public static void moveClimbersIndependent(double Rspeed, double Lspeed){ //moves each climber seperately, instead of together.
     Constants.climberMotorL.set(Functions.Clamp(Lspeed*(Constants.climberMotorLInvert?-1:1), 
    !Constants.lClimberSwitch.get()?0: Functions.Clamp(Constants.climberReductionMult*(lClimberAngle-Constants.climberSmoothingEnd)-Constants.climberMaxHitSpeed, -1, -Constants.climberMaxHitSpeed), 
    lClimberAngle>=Constants.climberMaxHeight?0:1));

    Constants.climberMotorR.set(Functions.Clamp(Rspeed*(Constants.climberMotorRInvert?-1:1), 
    !Constants.rClimberSwitch.get()?0:Functions.Clamp(Constants.climberReductionMult*(lClimberAngle-Constants.climberSmoothingEnd)-Constants.climberMaxHitSpeed, -1, -Constants.climberMaxHitSpeed), 
    rClimberAngle>=Constants.climberMaxHeight?0:1));
  }

  public static void moveClimbersTo(double rightpos, double leftpos, double speed){ //uses a P controller to move the climbers to a given angle.
    moveClimbersIndependent(Functions.Clamp((rightpos-rClimberAngle)*Constants.climberGoToPMult, -speed, speed), Functions.Clamp((leftpos-lClimberAngle)*Constants.climberGoToPMult, -speed, speed));
  }

  public static void autoBalance(double speed) { //uses a P controller to make the robot level with the ground, while still climbing onto the stage.
    moveClimbers(speed,Functions.Clamp((Constants.climberBalancePMult*(-Constants.gyro.getRoll().getValueAsDouble())), 
    -1, 1));
  }
}