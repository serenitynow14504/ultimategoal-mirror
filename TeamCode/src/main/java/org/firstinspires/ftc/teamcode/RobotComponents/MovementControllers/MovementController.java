package org.firstinspires.ftc.teamcode.RobotComponents.MovementControllers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Path;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

public abstract class MovementController {
    protected Path path;
    protected Robot robot;
    protected double progress;
    protected ElapsedTime timer;

    MovementController(Robot r, Path p) {
        path = p;
        robot = r;
    }

    public double getProgress() {
        return progress;
    }

    protected void setProgress(double progress) {
        this.progress = progress;
    }

    public abstract void followPath(double power);
}
