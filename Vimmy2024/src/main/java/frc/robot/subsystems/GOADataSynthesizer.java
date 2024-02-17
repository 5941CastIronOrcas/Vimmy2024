// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilityObjects.Obstacle;
import frc.robot.utilityObjects.SonarModule;
import frc.robot.utilityObjects.Vector2D;

public class GOADataSynthesizer extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static int upTimeFrames = 0;
  public static Obstacle[] staticObstacles = new Obstacle[] {
     new Obstacle(16.54-0.918718, 6.068568, true, Constants.subwooferAvoidanceMult, 0)
    ,new Obstacle(16.54-0, 6.53796, true, Constants.subwooferAvoidanceMult, 0)
    ,new Obstacle(16.54-0, 4.557776, true, Constants.subwooferAvoidanceMult, 0)
    ,new Obstacle(16.54-0.918718, 5.027168, true, Constants.subwooferAvoidanceMult, 0)
    //Blue subwoofer
    ,new Obstacle(0.918718, 6.068568, true, Constants.subwooferAvoidanceMult, 0)
    ,new Obstacle(0, 6.53796, true, Constants.subwooferAvoidanceMult, 0)
    ,new Obstacle(0, 4.557776, true, Constants.subwooferAvoidanceMult, 0)
    ,new Obstacle(0.918718, 5.027168, true, Constants.subwooferAvoidanceMult, 0)
    //Red stage
    ,new Obstacle(13.4666, 4.105, true, Constants.stageAvoidanceMult, 0)
    ,new Obstacle(10.769374, 2.547726, true, Constants.stageAvoidanceMult, 0)
    ,new Obstacle(10.769374, 5.662274, true, Constants.stageAvoidanceMult, 0)
    //Blue stage 
    ,new Obstacle(3.0734, 4.105, true, Constants.stageAvoidanceMult, 0)
    ,new Obstacle(5.770626, 2.547726, true, Constants.stageAvoidanceMult, 0)
    ,new Obstacle(5.770626, 5.662274, true, Constants.stageAvoidanceMult, 0)};
  public static Obstacle[] allObstacles = new Obstacle[(int)(Constants.maxObstacleLife * 50 * 8 + staticObstacles.length)];
  public static SonarModule[] SonarModules = new SonarModule[] {
     new SonarModule(Constants.ultrasonicSensor1, Constants.servo1, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor2, Constants.servo2, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor3, Constants.servo3, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor4, Constants.servo4, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor5, Constants.servo4, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor6, Constants.servo4, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor7, Constants.servo4, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor8, Constants.servo4, 0, 0, 0)};
    
    Vector2D[] tempSensorReturn = new Vector2D[8];

   
  public GOADataSynthesizer() {
    for (int i = 0; i < staticObstacles.length; i++) allObstacles[i] = new Obstacle(0, i, false, i, i); // merge the static into all
    
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 8; i++) tempSensorReturn[i] = SonarModules[i].GetObstaclePosition(); // get readings from sensors
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
