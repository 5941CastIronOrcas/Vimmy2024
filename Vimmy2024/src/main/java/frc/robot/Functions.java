package frc.robot;
import java.lang.Math;

import org.opencv.core.Mat;

import frc.robot.utilityObjects.Vector2D;


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
    public Vector2D Rotate(Vector2D VectorInpt, double AInpt) { return new Vector2D(Pythagorean(VectorInpt.x, VectorInpt.y) * Math.cos(Math.toRadians(AInpt) - Math.atan2(VectorInpt.y, VectorInpt.x)), Pythagorean(VectorInpt.y, VectorInpt.x) * Math.sin(Math.toRadians(-AInpt)+Math.atan2(VectorInpt.y, VectorInpt.x))); } 
    public static double DriverToFieldAngle(double angle)
    {
        if(Robot.isRedAlliance)
        {angle = angle - 90;}
        else if(Robot.isBlueAlliance)
        {angle = angle + 90;}
        return angle;
    }

    public static double FieldToDriverAngle(double angle)
    {
        if(Robot.isRedAlliance)
        {angle = angle + 90;}
        else if(Robot.isBlueAlliance)
        {angle = angle - 90;}
        return angle;
    }
    
    public static double AltAxisCoord(double x, double y, double a) //i dont know how to explain what this does, but it uses radians.
    {
        double s = Math.sin(a)*Math.cos(a)*(x*Cot(a)+y);
        return (Math.tan(a)*s<0?(Math.sin(a)>0?-1:1):(Math.sin(a)<0?-1:1))
        * Math.sqrt((s*s)+((Math.tan(a)*(s))*(Math.tan(a)*(s))));
    }
    public static double Cot(double a)
    {
        return Math.cos(a) / Math.sin(a);
    }

    public static Vector2D ClampVector(Vector2D in, double max)
    {
        double n = (Math.min(max, Pythagorean(in.x, in.y)))/(Pythagorean(in.x, in.y));
        return new Vector2D(n*in.x, n*in.y);
    }

    public static void setRobotLimp(boolean yes)
    {
        if(yes)
        {
            
        }
    }
}
