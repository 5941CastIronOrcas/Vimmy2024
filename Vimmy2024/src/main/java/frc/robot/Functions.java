package frc.robot;
import java.lang.Math;

import edu.wpi.first.wpilibj.DriverStation;

public class Functions {
    
    public static double DeadZone(double input, double deadZone) {
        if(Math.abs(input) < deadZone) 
        {
            return 0;
        }
        else 
        {
            return input;
        }
    }
    public static double Exponential(double input) {
        return input * Math.abs(input);
    }
    public static double Pythagorean(double x, double y) {
        return Math.sqrt((x * x) + (y * y));
    }
    public static double DeltaAngleDeg(double startAngle, double endAngle) {
        return((((endAngle - startAngle - 180) % 360) +360) % 360) - 180;
    }
    public static double DeltaAngleRadians(double startAngle, double endAngle) {
        return ((((endAngle - startAngle - Math.PI) % (2*Math.PI)) + (2*Math.PI)) % (2*Math.PI))-Math.PI;
    }
    public static double Clamp (double in, double min, double max) {
        return Math.min(Math.max(in,min),max);
    }
    public static double ClampMin (double in, double min) {
        return Math.max(in,min);
    }
    public static double ClampMax (double in, double max) {
        return Math.min(in,max);
    }
    public static double Max (double[]values) {
        double max = values[0];
        for (int i = 0; i < values.length; i++) {
            max = (values[i]>max)?values[i]:max;
        }
        return max;
    }
    public static double DriverToFieldAngle(double angle)
    {
        if(DriverStation.getAlliance().equals(DriverStation.Alliance.Red))
        {angle = Functions.DeltaAngleDeg(0, angle - 90);}
        else if(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue))
        {angle = Functions.DeltaAngleDeg(0, angle + 90);}
        return angle;
    }

    public static double FieldToDriverAngle(double angle)
    {
        if(DriverStation.getAlliance().equals(DriverStation.Alliance.Red))
        {angle = Functions.DeltaAngleDeg(0, angle + 90);}
        else if(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue))
        {angle = Functions.DeltaAngleDeg(0, angle - 90);}
        return angle;
    }
}
