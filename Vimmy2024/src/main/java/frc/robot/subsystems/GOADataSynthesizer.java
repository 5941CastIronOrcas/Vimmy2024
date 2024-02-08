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
  public static ArrayList<ArrayList<Obstacle>> staticObstacles = new ArrayList<ArrayList<Obstacle>>();
  public static ArrayList<ArrayList<Obstacle>> movingObstacles = new ArrayList<ArrayList<Obstacle>>();
  public GOADataSynthesizer() {
    //Red subwoofer
    ArrayList<Obstacle> redSubwoofer = new ArrayList<Obstacle>(); 
    redSubwoofer.add(new Obstacle(16.54-0.918718, 6.068568, true, Constants.subwooferAvoidanceMult));
    redSubwoofer.add(new Obstacle(16.54-0, 6.53796, true, Constants.subwooferAvoidanceMult));
    redSubwoofer.add(new Obstacle(16.54-0, 4.557776, true, Constants.subwooferAvoidanceMult));
    redSubwoofer.add(new Obstacle(16.54-0.918718, 5.027168, true, Constants.subwooferAvoidanceMult));
    //Blue subwoofer
    ArrayList<Obstacle> blueSubwoofer = new ArrayList<Obstacle>(); 
    blueSubwoofer.add(new Obstacle(0.918718, 6.068568, true, Constants.subwooferAvoidanceMult));
    blueSubwoofer.add(new Obstacle(0, 6.53796, true, Constants.subwooferAvoidanceMult));
    blueSubwoofer.add(new Obstacle(0, 4.557776, true, Constants.subwooferAvoidanceMult));
    blueSubwoofer.add(new Obstacle(0.918718, 5.027168, true, Constants.subwooferAvoidanceMult));
    //Red stage
    ArrayList<Obstacle> redStage = new ArrayList<Obstacle>(); 
    redStage.add(new Obstacle(13.4666, 4.105, true, Constants.subwooferAvoidanceMult));
    redStage.add(new Obstacle(10.769374, 2.547726, true, Constants.subwooferAvoidanceMult));
    redStage.add(new Obstacle(10.769374, 5.662274, true, Constants.subwooferAvoidanceMult));
    //Blue stage
    ArrayList<Obstacle> blueStage = new ArrayList<Obstacle>(); 
    blueStage.add(new Obstacle(3.0734, 4.105, true, Constants.subwooferAvoidanceMult));
    blueStage.add(new Obstacle(5.770626, 2.547726, true, Constants.subwooferAvoidanceMult));
    blueStage.add(new Obstacle(5.770626, 5.662274, true, Constants.subwooferAvoidanceMult));

    staticObstacles.add(redSubwoofer);
    staticObstacles.add(blueSubwoofer);
    staticObstacles.add(redStage);
    staticObstacles.add(blueStage);
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
