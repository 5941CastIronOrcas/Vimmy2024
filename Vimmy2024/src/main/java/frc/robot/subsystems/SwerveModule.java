package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import frc.robot.Functions;

import com.ctre.phoenix6.hardware.CANcoder;

/** The SwerveModule class takes the input angle and throttle from SwerveSubsystem, and rotates the angle and throttle motors.
 **/

public class SwerveModule {
    private CANSparkMax angleMotor;
    private CANSparkMax throttleMotor;
    private CANcoder encoder;
    private float aMult;
    private float tMult;
    public double anglePos;
    public double defaultAngle;
    public SwerveModule (CANSparkMax angleMotorIN, CANSparkMax throttleMotorIN, CANcoder encoderIN, boolean aMotorInvert, boolean tMotorInvert, double dAngle) {
        
        angleMotor = angleMotorIN;
        throttleMotor = throttleMotorIN;
        encoder = encoderIN;
        aMult = ((aMotorInvert)?-1:1);
        tMult = ((tMotorInvert)?-1:1);
        defaultAngle = dAngle;
        
    }
    public void Drive(double angle, double speed) {
        double throttle = speed;
        anglePos = encoder.getAbsolutePosition().getValueAsDouble() * 360;
        
        if (Math.abs(Functions.DeltaAngleDeg(anglePos, angle)) > Math.abs(Functions.DeltaAngleDeg(anglePos, angle + 180))) {
            angle += 180;
            throttle *= -1;
        }
        
        if(Math.abs(speed) < 0.001) {
            angle = defaultAngle;
        }
        
        angleMotor.set(Functions.Clamp(aMult*Functions.DeltaAngleDeg(angle, anglePos) * (Constants.modulePMult), -1,1));
        throttleMotor.set(throttle * tMult);
        
    }
}