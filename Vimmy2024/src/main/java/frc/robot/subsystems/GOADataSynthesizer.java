// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilityObjects.Obstacle;

public class GOADataSynthesizer extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ArrayList<Obstacle> staticObstacles = new ArrayList<Obstacle>();
  public ArrayList<Obstacle> movingObstacles = new ArrayList<Obstacle>();
  public GOADataSynthesizer() {
    //Red subwoofer
    staticObstacles.add(new Obstacle(16.54-0.918718, 6.068568, true, Constants.subwooferAvoidanceMult));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
