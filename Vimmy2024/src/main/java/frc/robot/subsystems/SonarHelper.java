// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilityObjects.SonarModule;

public class SonarHelper extends SubsystemBase {
  public double servoPanFrame = 0;
  public SonarHelper() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */

  @Override
  public void periodic() {
    if (Constants.isSonarModulesEnabled) {
    Constants.ultrasonicPing.set(true);
    for (SonarModule SonarModuleOnLoop : GOADataSynthesizer.SonarModules) SonarModuleOnLoop.UpdateSensorReading();
    Constants.ultrasonicPing.set(false);
    double localServoAngleToSet = 0.5 * Math.sin(servoPanFrame) + 0.5;
    for (SonarModule SonarModuleOnLoop : GOADataSynthesizer.SonarModules) SonarModuleOnLoop.SetServoAngle(localServoAngleToSet);
    servoPanFrame += Constants.ServoSpeed;
    
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
