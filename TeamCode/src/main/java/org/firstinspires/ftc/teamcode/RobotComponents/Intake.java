package org.firstinspires.ftc.teamcode.RobotComponents;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

class Intake extends Capability {
    private ExpansionHubMotor intakeWheels;
    private final double CURRENT_THRESHOLD = 10;

    public Intake(Robot parent) {
        super(parent);
    }

    public void init(HardwareMap hardwareMap) {
        intakeWheels = (ExpansionHubMotor)hardwareMap.get("intake");
    }

    public void teleOp(Gamepad gamepad) {
        //intakeWheels.setPower(gamepad.right_stick_y);
        if(gamepad.dpad_right) {
            on();
            parent.setLedColors(0,255,0);
        } else if(gamepad.dpad_left) {
            off();
        } else if(gamepad.y) {
            reverse();
        }
    }

    public void on() {
        intakeWheels.setPower(1);
    }

    public void reverse() {
        intakeWheels.setPower(-1);
    }

    public void off() {
        intakeWheels.setPower(0);
    }

    public void toggle() {
        if(intakeWheels.getPower()>0) {
            on();
        } else {
            off();
        }
    }

    @Override
    public void run() {
        while(opModeIsActive()) {
            if (intakeWheels.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > CURRENT_THRESHOLD) {
                intakeWheels.setPower(0);
                parent.setLedColors(255, 0, 0);
            }
        }
    }
}
