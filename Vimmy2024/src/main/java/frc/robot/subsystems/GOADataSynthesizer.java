// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilityObjects.Obstacle;
import frc.robot.utilityObjects.SonarModule;

public class GOADataSynthesizer extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static ArrayList<Obstacle> staticObstacles = new ArrayList<Obstacle>();
  public static ArrayList<Obstacle> movingObstacles = new ArrayList<Obstacle>();
  public static ArrayList<Obstacle> allObstacles = new ArrayList<Obstacle>();
  public static ArrayList<SonarModule> SonarModules = new ArrayList<SonarModule>();
  public GOADataSynthesizer() {
    //Sonar modules
    SonarModules.add(new SonarModule(Constants.ultrasonicSensor1, Constants.servo1, 0, 0, 0));
    SonarModules.add(new SonarModule(Constants.ultrasonicSensor2, Constants.servo2, 0, 0, 0));
    SonarModules.add(new SonarModule(Constants.ultrasonicSensor3, Constants.servo3, 0, 0, 0));
    SonarModules.add(new SonarModule(Constants.ultrasonicSensor4, Constants.servo4, 0, 0, 0));
    SonarModules.add(new SonarModule(Constants.ultrasonicSensor5, Constants.servo4, 0, 0, 0));
    SonarModules.add(new SonarModule(Constants.ultrasonicSensor6, Constants.servo4, 0, 0, 0));
    SonarModules.add(new SonarModule(Constants.ultrasonicSensor7, Constants.servo4, 0, 0, 0));
    SonarModules.add(new SonarModule(Constants.ultrasonicSensor8, Constants.servo4, 0, 0, 0));
    //Red subwoofer
    staticObstacles.add(new Obstacle(16.54-0.918718, 6.068568, true, Constants.subwooferAvoidanceMult));
    staticObstacles.add(new Obstacle(16.54-0, 6.53796, true, Constants.subwooferAvoidanceMult));
    staticObstacles.add(new Obstacle(16.54-0, 4.557776, true, Constants.subwooferAvoidanceMult));
    staticObstacles.add(new Obstacle(16.54-0.918718, 5.027168, true, Constants.subwooferAvoidanceMult));
    //Blue subwoofer
    staticObstacles.add(new Obstacle(0.918718, 6.068568, true, Constants.subwooferAvoidanceMult));
    staticObstacles.add(new Obstacle(0, 6.53796, true, Constants.subwooferAvoidanceMult));
    staticObstacles.add(new Obstacle(0, 4.557776, true, Constants.subwooferAvoidanceMult));
    staticObstacles.add(new Obstacle(0.918718, 5.027168, true, Constants.subwooferAvoidanceMult));
    //Red stage
    staticObstacles.add(new Obstacle(13.4666, 4.105, true, Constants.stageAvoidanceMult));
    staticObstacles.add(new Obstacle(10.769374, 2.547726, true, Constants.stageAvoidanceMult));
    staticObstacles.add(new Obstacle(10.769374, 5.662274, true, Constants.stageAvoidanceMult));
    //Blue stage 
    staticObstacles.add(new Obstacle(3.0734, 4.105, true, Constants.stageAvoidanceMult));
    staticObstacles.add(new Obstacle(5.770626, 2.547726, true, Constants.stageAvoidanceMult));
    staticObstacles.add(new Obstacle(5.770626, 5.662274, true, Constants.stageAvoidanceMult));

    for (int i = 0; i < staticObstacles.size(); i++) allObstacles.add(staticObstacles.get(i));
    for (int i = 0; i < movingObstacles.size(); i++) allObstacles.add(movingObstacles.get(i));
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
