package frc.robot.utilityObjects;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.Functions;
import frc.robot.subsystems.PositionEstimator;

public class SonarModule {
    static AnalogInput LocalSonic;
    static Servo LocalServo;
    static double Rotation;
    static double Xoffset;
    static double Yoffset;
    static double LastReeding;

    public SonarModule(AnalogInput SonicInpt, Servo ServoInpt, double RotationInpt, double XoffsetInpt, double YoffsetInpt) {
        LocalSonic = SonicInpt;
        LocalServo = ServoInpt;
        Rotation = RotationInpt; //The robot-relative angle that the sensor is facing whem the servo is at 0 degrees (rotated to the max counterclockwise)
        Xoffset = XoffsetInpt;
        Yoffset = YoffsetInpt;
    }

    public void SetServoAngle(double ServoAngleToSet) {LocalServo.set(ServoAngleToSet * Constants.servoFinalMult);} 
    public void SetServoAngleDegrees(double ServoAngleToSet) {LocalServo.set(ServoAngleToSet / 360);}
    public double GetObstacleDistance() {
        // "data outside of or belonging to the wall" - not clear how to do that
        return Functions.Clamp(LastReeding, Constants.SonicMinValue, Constants.SonicMaxValue);
    }// max counter clockvise is 0 
    // the return is in degrees
    public double GetObstacleAngle() {return LocalServo.getAngle() + Rotation;}
    
    public Vector2D GetObstaclePosition() {
        
        double XsensorRelative = Math.cos(GetObstacleAngle()) * GetObstacleDistance();
        double YsensorRelative = Math.sin(GetObstacleAngle()) * GetObstacleDistance();
        
        double XrobotRelative = XsensorRelative + Xoffset;
        double YrobotRelative = YsensorRelative + YsensorRelative;
        Vector2D AlmostFieldRelative = Functions.Rotate(new Vector2D(XrobotRelative, YrobotRelative), PositionEstimator.robotPosition.getRotation().getDegrees());
        Vector2D FieldRelative = new Vector2D(AlmostFieldRelative.x + PositionEstimator.robotPosition.getX(), AlmostFieldRelative.y + PositionEstimator.robotPosition.getY());
        return FieldRelative;
    }
    
    public void UpdateSensorReading() {LastReeding = LocalSonic.getValue()*Constants.voltageScaleFactor*0.125;}

    public void UpdateVoltageScaleFactor() {Constants.voltageScaleFactor = 5/RobotController.getVoltage5V();}

}
