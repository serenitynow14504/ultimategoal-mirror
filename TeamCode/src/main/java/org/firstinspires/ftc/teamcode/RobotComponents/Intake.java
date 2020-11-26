package org.firstinspires.ftc.teamcode.RobotComponents;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

class Intake extends Capability {
    private ExpansionHubMotor intakeWheels;
    private DistanceSensor stuckSensor;
    private final double CURRENT_THRESHOLD = 10;
    private ElapsedTime stuckTime;

    public Intake(Robot parent) {
        super(parent);
    }

    public void init(HardwareMap hardwareMap) {
        intakeWheels = (ExpansionHubMotor)hardwareMap.get("intake");
        stuckSensor = hardwareMap.get(DistanceSensor.class, "ramp");
        stuckTime = new ElapsedTime();
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
        intakeWheels.setPower(-0.5);
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

    public boolean ringStuck() {
        return sensorDist() < 15;
    }

    public double sensorDist() {
        return stuckSensor.getDistance(DistanceUnit.MM);
    }

    private void reTakeRing() {
        off();
        sleep(200);
        while(ringStuck()) {
            reverse();
        }
        off();
        sleep(100);
        on();
    }

    @Override
    public void run() {
        stuckTime.reset();
        while(opModeIsActive()) {
            if(ringStuck()) {
                if(stuckTime.milliseconds()>500) {
                    reTakeRing();
                }
            } else {
                stuckTime.reset();
            }
            if (intakeWheels.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > CURRENT_THRESHOLD) {
                intakeWheels.setPower(0);
                parent.setLedColors(255, 0, 0);
            }
        }
    }
}
