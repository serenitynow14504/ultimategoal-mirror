package org.firstinspires.ftc.teamcode.RobotComponents.MovementControllers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Path;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

public abstract class MovementController {
    protected Path path;
    protected Robot robot;
    protected double progress, power;
    protected long timeOutMillis;
    protected ElapsedTime timer;
    private boolean running = false;

    MovementController(Robot r, Path p) {
        robot = r;
        path = p;
    }

    public double getProgress() {
        return progress;
    }

    protected void setProgress(double progress) {
        this.progress = progress;
    }

    public void followPath(double power) {
        followPath(power, 30000);
    }

    public abstract void followPath(double power, long timeOutMillis);

}
