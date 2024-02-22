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
  public static short currentFrame = 0;
  public static int upTimeFrames = 0;
  public static short ignoranceID = 0;
  public static short PointerCluster = 0;
  public static Obstacle[] staticObstacles = new Obstacle[] {
     new Obstacle(16.54-0.918718, 6.068568, true, Constants.subwooferAvoidanceMult)
    ,new Obstacle(16.54-0, 6.53796, true, Constants.subwooferAvoidanceMult)
    ,new Obstacle(16.54-0, 4.557776, true, Constants.subwooferAvoidanceMult)
    ,new Obstacle(16.54-0.918718, 5.027168, true, Constants.subwooferAvoidanceMult)
    //Blue subwoofer
    ,new Obstacle(0.918718, 6.068568, true, Constants.subwooferAvoidanceMult)
    ,new Obstacle(0, 6.53796, true, Constants.subwooferAvoidanceMult)
    ,new Obstacle(0, 4.557776, true, Constants.subwooferAvoidanceMult)
    ,new Obstacle(0.918718, 5.027168, true, Constants.subwooferAvoidanceMult)
    //Red stage
    ,new Obstacle(13.4666, 4.105, true, Constants.stageAvoidanceMult)
    ,new Obstacle(10.769374, 2.547726, true, Constants.stageAvoidanceMult)
    ,new Obstacle(10.769374, 5.662274, true, Constants.stageAvoidanceMult)
    //Blue stage 
    ,new Obstacle(3.0734, 4.105, true, Constants.stageAvoidanceMult)
    ,new Obstacle(5.770626, 2.547726, true, Constants.stageAvoidanceMult)
    ,new Obstacle(5.770626, 5.662274, true, Constants.stageAvoidanceMult)};
    ignoranceID = staticObstacles.length;
  public static Obstacle[] allObstacles = new Obstacle[(int) (Constants.maxObstacleLifeFrames / Constants.GOAFrameRate) * 8 + ignoranceID];
  public static SonarModule[] SonarModules = new SonarModule[] {
     new SonarModule(Constants.ultrasonicSensor1, Constants.servo1, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor2, Constants.servo2, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor3, Constants.servo3, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor4, Constants.servo4, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor5, Constants.servo4, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor6, Constants.servo4, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor7, Constants.servo4, 0, 0, 0)
    ,new SonarModule(Constants.ultrasonicSensor8, Constants.servo4, 0, 0, 0)};
  
    for (int i = 0; i < ignoranceID; i++) allObstacles[i] = staticObstacles[i]; // merge the static into all

    Vector2D[] tempSensorReturn = new Vector2D[8];

   
  public GOADataSynthesizer() {

  }

  @Override
  public void periodic() {
    for (int i = 0; i < 8; i++) tempSensorReturn[i] = SonarModules[i].GetObstaclePosition(); // get readings from sensors
    if (currentFrame >= Constants.GOAFrameRate) {
    //update the one frame cluster data
    for (int i = 0; i < 8; i++) {
      allObstacles[i] = new Obstacle(SonarModules[PointerCluster * 8 + ignoranceID + i].GetObstaclePosition().x, SonarModules[PointerCluster + ignoranceID--].GetObstaclePosition().y, false, 1);
    }
    PointerCluster = (short) (PointerCluster++ % (Constants.maxObstacleLifeFrames / Constants.GOAFrameRate));
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
