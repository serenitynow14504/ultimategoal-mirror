package org.firstinspires.ftc.teamcode.robotComponents;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Capability extends Thread {
    Robot robot;

    public Capability(Robot r) {
        robot = r;
    }

    boolean isStopRequested() {
        return robot.getMyOpMode().isStopRequested();
    }

    boolean opModeIsActive() {
        return robot.getMyOpMode().opModeIsActive();
    }

    void sleep(int millis) {
        robot.getMyOpMode().sleep(millis);
    }


    abstract void teleOp(Gamepad gamepad1, Gamepad gamepad2);
    void debugOp(Gamepad gamepad1, Gamepad gamepad2) {
        teleOp(gamepad1, gamepad2);
    }
    abstract void init(HardwareMap hardwareMap);
    abstract void tick(TelemetryPacket packet);
    public void tickInit() {}
}
