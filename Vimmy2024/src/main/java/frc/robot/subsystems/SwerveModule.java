package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public double velocity;
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
        //anglePos = encoder.getAbsolutePosition().getValueAsDouble() * 360;
        anglePos = Functions.DeltaAngleDeg(0, encoder.getPosition().getValueAsDouble() * 360);
        velocity = throttleMotor.getEncoder().getVelocity() + (encoder.getVelocity().getValueAsDouble()*(27.0 / 17.0)*(1.0/3.0)*60);
        SmartDashboard.putNumber("wheel velocity", velocity);
        SmartDashboard.putNumber("angle velocity", encoder.getVelocity().getValueAsDouble());

        if(Math.abs(speed) < 0.001) {
            //angle = anglePos;
            angle = defaultAngle;
        }

        if (Math.abs(Functions.DeltaAngleDeg(anglePos, angle)) > Math.abs(Functions.DeltaAngleDeg(anglePos, angle + 180))) {
            angle += 180;
            throttle *= -1;
        }
        //double compensationSpeed = -encoder.getVelocity().getValueAsDouble()*(27.0 / 17.0)*(1.0/3.0)*Constants.swerveDriveRatio*60;
        //double currentSpeed = throttleMotor.getEncoder().getVelocity();
        angleMotor.set(Functions.Clamp(aMult*Functions.DeltaAngleDeg(angle, anglePos) * (Constants.modulePMult), -1,1));
       // throttleMotor.set(throttleMotor.get()+((compensationSpeed-currentSpeed)*0.00001));
       // SmartDashboard.putNumber("Throttle motor power", throttleMotor.get());
        //SmartDashboard.putNumber("Throttle motor rpm", currentSpeed);
       // SmartDashboard.putNumber("Target speed", compensationSpeed);
        //SmartDashboard.putNumber("Target velocity", (compensationSpeed-currentSpeed)*0.00001);
        throttleMotor.set(throttle * tMult);
        
    }
}