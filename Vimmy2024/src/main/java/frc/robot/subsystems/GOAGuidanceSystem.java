// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.utilityObjects.Obstacle;
import frc.robot.utilityObjects.Vector2D;

public class GOAGuidanceSystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public GOAGuidanceSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Vector2D GetAvoidanceVector()
  {
    Vector2D robotPosition = new Vector2D(PositionEstimator.robotPosition.getX(), PositionEstimator.robotPosition.getY());
    ArrayList<Obstacle> relevantObstacles = new ArrayList<Obstacle>();
    for(int i = 0; i < relevantObstacles.size(); i++)
    {
      if(Functions.Pythagorean(relevantObstacles.get(i).x-robotPosition.x, relevantObstacles.get(i).y-robotPosition.y) > Constants.GOAIgnoreRange)
      {
        relevantObstacles.remove(i--);
      }
    }
    double[] obstacleAngles = new double[relevantObstacles.size()];
    double[] obstacleDistances = new double[relevantObstacles.size()];
    double[] obstacleRelativeVelocities = new double[relevantObstacles.size()];
    for(int i = 0; i < obstacleAngles.length; i++)
    {
      obstacleAngles[i] = Math.toDegrees(Math.atan2(relevantObstacles.get(i).x-robotPosition.x, relevantObstacles.get(i).y-robotPosition.y));
      obstacleDistances[i] = Functions.Pythagorean(relevantObstacles.get(i).x - robotPosition.x, relevantObstacles.get(i).y - robotPosition.y);
      obstacleRelativeVelocities[i] = Functions.AltAxisCoord(PositionEstimator.velocity.x, PositionEstimator.velocity.y, Math.toRadians(90-obstacleAngles[i]));
    }

    Vector2D out = new Vector2D(0, 0);
    for(int i  = 0; i < obstacleAngles.length; i++)
    {
      double avoidStrength = ((Constants.avoidanceMult*relevantObstacles.get(i).avoidanceMult) + (Constants.VelocityAvoidanceMult * obstacleRelativeVelocities[i])) / Math.pow(obstacleDistances[i], Constants.avoidanceExponent);
      out.x += -avoidStrength * Math.sin(Math.toRadians(obstacleAngles[i]));
      out.y += -avoidStrength * Math.cos(Math.toRadians(obstacleAngles[i]));
    }

    return out;
  }
  public static double GetAvoidanceVectorX()
  {
    return GetAvoidanceVector().x;
  }
  public static double GetAvoidanceVectorY()
  {
    return GetAvoidanceVector().y;
  }

  public static Vector2D GetDriveVector(double x, double y)
  {
    Vector2D robotPosition = new Vector2D(PositionEstimator.robotPosition.getX(), PositionEstimator.robotPosition.getY());
    Vector2D avoid = GetAvoidanceVector();
    double toTargetStrengthUncapped = Constants.swerveDriveToPMult*Functions.DeadZone(Functions.Pythagorean(x-PositionEstimator.robotPosition.getX(), y-PositionEstimator.robotPosition.getY()), Constants.swerveDriveToDeadZone);
    double toTargetStrength = toTargetStrengthUncapped>Constants.toTargetStrengthTop?Constants.toTargetStrengthCap:toTargetStrengthUncapped;
    double toTargetAngle = Math.atan2(x-robotPosition.x, y-robotPosition.y);
    Vector2D toTargetVector = new Vector2D(toTargetStrength*Math.sin(toTargetAngle), toTargetStrength*Math.cos(toTargetAngle));
    Vector2D driveVector = new Vector2D(toTargetVector.x+avoid.x, toTargetVector.y+avoid.y);
    return Functions.ClampVector(driveVector, 1);
  }
  public static double GetDriveVectorX(double x, double y)
  {
    return GetDriveVector(x, y).x;
  }
  public static double GetDriveVectorY(double x, double y)
  {
    return GetDriveVector(x, y).y;
  }

  public static double GetProtectionAngle()
  {
    Vector2D a = GetAvoidanceVector();
    return Math.toDegrees(Math.atan2(a.x, a.y));
  }
}
