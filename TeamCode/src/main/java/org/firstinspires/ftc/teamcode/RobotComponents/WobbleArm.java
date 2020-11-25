package org.firstinspires.ftc.teamcode.RobotComponents;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import androidx.annotation.NonNull;

public class WobbleArm extends Capability {
    private Servo armL, armR, pin;
    boolean armState = false, grabberState = false;

    public WobbleArm(Robot parent) {
        super(parent);
    }


    public void init(HardwareMap hardwareMap) {
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        pin = hardwareMap.get(Servo.class, "grabber");
    }

    public void teleOp(@NonNull Gamepad gamepad) {
        if(!gamepad.left_bumper) {
            if (gamepad.dpad_up) {
                arm(true);
            } else if (gamepad.dpad_down) {
                arm(false);
            }
        } else {
            if (gamepad.dpad_up) {
                grabber(true);
            } else if (gamepad.dpad_down) {
                grabber(false);
            }
        }
    }

    public void arm(boolean state) {
        if(state) {
            armL.setPosition(0);
            armR.setPosition(0.91);
        } else {
            armL.setPosition(0.4);
            armR.setPosition(0.565);
        }
        armState = state;
    }

    public void grabber(boolean state) {
        if(state) {
            pin.setPosition(1);
        } else {
            pin.setPosition(0.4);
        }
        grabberState = state;
    }
}
