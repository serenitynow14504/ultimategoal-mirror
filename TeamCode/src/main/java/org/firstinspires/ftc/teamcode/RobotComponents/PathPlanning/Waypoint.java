package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning;

public class Waypoint {
    private double xPos, yPos, rot;
    public enum RotMode{
        INTERPOLATE, TANGENT
    }
    public Waypoint(double x, double y, RotMode r) {
        xPos = x;
        yPos = y;
    }

    public Waypoint(double x, double y, double r) {
        xPos = x;
        yPos = y;
        rot = r;
    }
}
