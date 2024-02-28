// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ControllerRumble extends SubsystemBase {
  public static double controller1RumbleL;
  public static double controller1RumbleR;
  public static double controller2RumbleL;
  public static double controller2RumbleR;

  public static double controller1RumbleLtarget;
  public static double controller1RumbleRtarget;
  public static double controller2RumbleLtarget;
  public static double controller2RumbleRtarget;
    /** Creates a new ExampleSubsystem. */
  public ControllerRumble() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Constants.controller1.setRumble(RumbleType.kLeftRumble, controller1RumbleL);
    Constants.controller1.setRumble(RumbleType.kRightRumble, controller1RumbleR);
    Constants.controller2.setRumble(RumbleType.kLeftRumble, controller2RumbleL);
    Constants.controller2.setRumble(RumbleType.kRightRumble, controller2RumbleR);
    if(ArmSubsystem.hasNote)
    {
      RumbleBothControllersBothSides(1.0);
    }
    else
    {
      RumbleBothControllersBothSides(0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static void RumbleBothControllersBothSides(double intensity) {
    controller1RumbleL = intensity;
    controller1RumbleR = intensity;
    controller2RumbleL = intensity;
    controller2RumbleR = intensity;
  }

  public static void smoothTransition1L(double step, double target) {
    controller1RumbleLtarget = target;
    //controller1RumbleL = controller1RumbleL != 
  }

  public static void smoothTransition1R(double step, double target) {
    controller1RumbleRtarget = target;
  }
  
  public static void smoothTransition2L(double step, double target) {
    controller2RumbleLtarget = target;
  }
  
  public static void smoothTransition2R(double step, double target) {
    controller2RumbleRtarget = target;
  }
  
}
