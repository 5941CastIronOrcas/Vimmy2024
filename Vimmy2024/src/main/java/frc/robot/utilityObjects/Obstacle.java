package frc.robot.utilityObjects;

public class Obstacle {
    public double x;
    public double y;
    public boolean isStatic;
    public double avoidanceMult;
    public double birthTime;
    public Obstacle(double nx, double ny, boolean nIsStatic, double nAvoidanceMult, double nBirthTime) {
        x = nx;
        y = ny;
        isStatic = nIsStatic;
        avoidanceMult = nAvoidanceMult;
        birthTime = nBirthTime;
    }
}
