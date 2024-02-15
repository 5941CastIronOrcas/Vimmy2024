package frc.robot.utilityObjects;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.Functions;

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
        Rotation = RotationInpt;
        Xoffset = XoffsetInpt;
        Yoffset = YoffsetInpt;
    }

    public void SetServoAngle(double ServoAngleToSet) {LocalServo.set(ServoAngleToSet * Constants.servoFinalMult);} 
    
    public double GetObstacleDistance() {
        // "data outside of or belonging to the wall" - not clear how to do that
        return Functions.Clamp(LastReeding, Constants.SonicMinValue, Constants.SonicMaxValue);
    }
    // the return is in degrees
    public double GetObstacleAngle() {return LocalServo.getAngle();}

    public Vector2D GetObstaclePosition() {
        double fieldRealtiveRotation = Rotation + GetObstacleAngle(); //the rotation is the rotation of whole robot
                
        double XrelativeToCenter = 0;
        double YrealtiveToCenter = 0;
        //servos move different than 180 it's 210 ish
        double XfieldRelative = 0;
        double YfieldRelative = 0;
        return new Vector2D(XfieldRelative, YfieldRelative);
    }
    
    public void UpdateSensorReading() {LastReeding = LocalSonic.getValue()*Constants.voltageScaleFactor*0.125;}

    public void UpdateVoltageScaleFactor() {Constants.voltageScaleFactor = 5/RobotController.getVoltage5V();}

}
