package org.firstinspires.ftc.teamcode.RobotComponents;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;

public abstract class Capability extends Thread {
    Robot parent;

    boolean busy = true;
    protected double progress = 0d;

    protected Telemetry telemetry;



    public Capability(Robot r) {
        parent = r;
        telemetry = r.getMyOpMode().telemetry;
    }

    boolean isStopRequested() {
        return parent.getMyOpMode().isStopRequested();
    }


    boolean opModeIsActive() {
        return parent.getMyOpMode().opModeIsActive();
    }

    public double getProgress() {
        return progress;
    }

    public void setProgress(double progress) {
        this.progress = progress;
    }

    public boolean amIBusy() {
        return this.busy;
    }

    void sleep(int millis) {
        parent.getMyOpMode().sleep(millis);
    }


    int inchesToEncoderCounts(double inches) {
        return (int)(RobotConstants.INCHES_TO_COUNTS_CONVERSION * inches);
        //return (int) (inches * Robot.ODO_ENCODER_COUNTS_PER_REV / Robot.ODO_WHEEL_CIRC);
    }

    double encoderCountsToInches(int counts) {
        return RobotConstants.COUNTS_TO_INCHES_CONVERSION * counts;
        //return ((double)counts/Robot.ODO_ENCODER_COUNTS_PER_REV) * Robot.ODO_WHEEL_CIRC;
    }

    @Override
    public void run() {

    }

    abstract void teleOp(Gamepad gamepad1, Gamepad gamepad2);
    abstract void init(HardwareMap hardrwareMap);
}
