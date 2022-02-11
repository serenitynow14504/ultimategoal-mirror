package org.firstinspires.ftc.teamcode.robotComponents.MovementControllers;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.VectorD;
import org.firstinspires.ftc.teamcode.robotComponents.PathPlanning.Path;
import org.firstinspires.ftc.teamcode.robotComponents.Powers.WheelPowers;
import org.firstinspires.ftc.teamcode.robotComponents.Robot;

public abstract class MovementController {
    protected Path path;
    @Deprecated
    protected Robot robot;
    protected WheelPowers powers;
    protected double progress, power;
    protected long timeOutMillis;
    protected ElapsedTime timer;
    private boolean running = false;

    MovementController(WheelPowers powers, Robot robot) {
        this.powers = powers;
        this.robot = robot;
        timer = new ElapsedTime();
    }

    public double getProgress() {
        return progress;
    }

    protected void setProgress(double progress) {
        this.progress = progress;
    }

    public void followPath(Path path, double power) {
        this.path = path;
        progress = 0;
        this.power = power;
        running = true;
        doInit();
        timer.reset();
    }

    public void tick(VectorD pose, TelemetryPacket packet) {
        if(timer.milliseconds() > timeOutMillis) {
            running = false;
        }
        if(!running) {
            return;
        }

        if(shouldExit(pose)) {
            doFinish();
            running = false;
        } else {
            doTick(pose, packet);
        }

    }

    @Deprecated
    public abstract void followPath(double power, long timeOutMillis);

    protected abstract void doTick(VectorD pose, TelemetryPacket packet);
    protected abstract void doInit();
    protected abstract void doFinish();
    protected abstract boolean shouldExit(VectorD pose);

    public void setTimeOut(long millis) {
        timeOutMillis = millis;
    }

    public void noTimeOut() {
        setTimeOut(30000);
    }
}
