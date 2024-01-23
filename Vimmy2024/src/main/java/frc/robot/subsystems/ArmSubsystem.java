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
  
  public double raiseAngle1; //current raise angle of the arm, in degrees. Horizontal is 0 degrees, down is negative, up is positive. 
  public double raiseAngle2; //angle of second arm relative to the horizon. 
  public double bendAngle;
  public double minClawAngle;
  public double maxClawAngle;
  
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {}

  @Override
  public void periodic() {
    raiseAngle1 = Constants.armMotor1.getEncoder().getPosition() * 360 * Constants.armGearRatio1; //sets current raise angle to encoder positions.
    raiseAngle2 = (Constants.armMotor2.getEncoder().getPosition() * 360 * Constants.armGearRatio2); //if it doesn't work check the 180
    bendAngle = raiseAngle2 - raiseAngle1; //sets the current bend angle based on the encoder positions.
    minClawAngle = raiseAngle1-90;
    maxClawAngle = raiseAngle1+90;
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void moveArmTo(double a1, double a2) {
    double a2Clamped = Functions.Clamp(a2, minClawAngle, maxClawAngle);
    rotateArm(Functions.Clamp((Constants.armMotor1PMult * (a1 - raiseAngle1))
    +(Constants.armMotor1GravMult * Math.cos(Math.toRadians(raiseAngle1)))
    +(Constants.armMotor1DMult*Constants.armMotor1.getEncoder().getVelocity()), 
    -Constants.maxArmSpeed, Constants.maxArmSpeed), 
            Functions.Clamp((Constants.armMotor2PMult * (a2Clamped - raiseAngle2))
    +(Constants.armMotor2GravMult * Math.cos(Math.toRadians(raiseAngle2)))
    +(Constants.armMotor2DMult*Constants.armMotor2.getEncoder().getVelocity()), 
    -Constants.maxArmSpeed, Constants.maxArmSpeed)
    );
    SmartDashboard.putNumber("Arm Motor 1 Target", a1);
    SmartDashboard.putNumber("Arm Motor 2 Target", a2Clamped);
  }
  
  public void rotateArm(double t1, double t2) {
    Constants.armMotor1.set((Constants.armMotor1Invert)?-t1:t1);
    Constants.armMotor2.set((Constants.armMotor2Invert)?-t2:t2);
    SmartDashboard.putNumber("Arm Motor 1 Throttle", (Constants.armMotor1Invert)?-t1:t1);
    SmartDashboard.putNumber("Arm Motor 2 Throttle", (Constants.armMotor2Invert)?-t2:t2);
  }
}
