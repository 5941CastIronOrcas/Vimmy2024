package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import frc.robot.Functions;

import com.ctre.phoenix.sensors.WPI_CANCoder;
public class SwerveModule {
    private CANSparkMax angleMotor;
    private CANSparkMax throttleMotor;
    private WPI_CANCoder encoder;
    private float aMult;
    private float tMult;
    public SwerveModule (CANSparkMax angleMotorIN, CANSparkMax throttleMotorIN, WPI_CANCoder encoderIN, boolean aMotorInvert, boolean tMotorInvert) {
        
        angleMotor = angleMotorIN;
        throttleMotor = throttleMotorIN;
        encoder = encoderIN;
        aMult = ((aMotorInvert)?-1:1);
        tMult = ((tMotorInvert)?-1:1);
        
    }
    public void Drive(double angle, double speed) {
        double throttle = speed;
        
        if (Math.abs(Functions.DeltaAngleDeg(encoder.getPosition(), angle)) > Math.abs(Functions.DeltaAngleDeg(encoder.getPosition(), angle + 180))) {
            angle += 180;
            throttle *= -1;
        }
        
        if(Math.abs(Functions.DeltaAngleDeg(angle, encoder.getPosition())) > 10) {
            throttle = 0;
        }
        
        if(Math.abs(speed) < 0.001) {
            angle = encoder.getPosition();
        }
        
        angleMotor.set(Functions.Clamp(Functions.DeltaAngleDeg(angle, encoder.getPosition()) * -(Constants.modulePMult),-Constants.maxTurnSpeed,Constants.maxTurnSpeed) * aMult);
        throttleMotor.set(throttle * tMult);
    }
}