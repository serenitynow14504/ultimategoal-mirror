package org.firstinspires.ftc.teamcode.RobotComponents.Tasks;

public abstract class ExtentPowerTask extends Task {
    protected double extent, power;

    public ExtentPowerTask(double extent, double power) {
        this.extent = extent;
        this.power = power;
    }
}
