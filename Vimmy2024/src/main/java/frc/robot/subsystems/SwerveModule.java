package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import frc.robot.Functions;

import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
    private CANSparkMax angleMotor;
    private CANSparkMax throttleMotor;
    private CANcoder encoder;
    private float aMult;
    private float tMult;
    public double anglePos;
    public SwerveModule (CANSparkMax angleMotorIN, CANSparkMax throttleMotorIN, CANcoder encoderIN, boolean aMotorInvert, boolean tMotorInvert) {
        
        angleMotor = angleMotorIN;
        throttleMotor = throttleMotorIN;
        encoder = encoderIN;
        aMult = ((aMotorInvert)?-1:1);
        tMult = ((tMotorInvert)?-1:1);
        
    }
    public void Drive(double angle, double speed) {
        double throttle = speed;
        anglePos = encoder.getAbsolutePosition().getValueAsDouble();
        
        if (Math.abs(Functions.DeltaAngleDeg(encoder.getAbsolutePosition().getValueAsDouble() * 360, angle)) > Math.abs(Functions.DeltaAngleDeg(encoder.getAbsolutePosition().getValueAsDouble() * 360, angle + 180))) {
            angle += 180;
            throttle *= -1;
        }
        
        if(Math.abs(speed) < 0.001) {
            angle = encoder.getAbsolutePosition().getValueAsDouble() * 360;
        }
        
        angleMotor.set(Functions.Clamp(Functions.DeltaAngleDeg(angle, encoder.getAbsolutePosition().getValueAsDouble() * 360) * -(Constants.modulePMult),-aMult,aMult));
        throttleMotor.set(throttle * tMult);
        
    }
}