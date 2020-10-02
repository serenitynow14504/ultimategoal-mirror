package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

public class doubleDCMotor {
    private DcMotor one, two;
    private boolean oneRev, twoRev;//whether they are reversed
    private Robot parent;
    private LinearOpMode parentOpMode;

    public doubleDCMotor(Robot r, boolean rev1, boolean rev2) {
        parent = r;
        oneRev = rev1;
        twoRev = rev2;
    }

    public doubleDCMotor(LinearOpMode l, boolean rev1, boolean rev2) {
        parentOpMode = l;
        oneRev = rev1;
        twoRev = rev2;
    }

    public void init(DcMotor one, DcMotor two) {
        this.one = one;
        this.two = two;

        if(oneRev) {
            one.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            one.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        if(twoRev) {
            two.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            two.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void setPower(double pow) {
        one.setPower(pow);
        two.setPower(pow);
    }

    public double getPower() {
        return one.getPower();
    }

    public void setMode(DcMotor.RunMode r) {
        one.setMode(r);
        two.setMode(r);
    }

    public boolean isBusy() {
        return one.isBusy() || two.isBusy();
    }

    public void setTargetPosition(int position) {
        one.setTargetPosition(position);
        two.setTargetPosition(position);
    }

    public DcMotor.RunMode getMode() {
        return one.getMode();
    }

    public int getTargetPosition() {
        return one.getTargetPosition();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        one.setZeroPowerBehavior(zeroPowerBehavior);
        two.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return one.getZeroPowerBehavior();
    }

    public int getCurrentPosition() {
        return (one.getCurrentPosition() + two.getCurrentPosition())/2;
    }
}
