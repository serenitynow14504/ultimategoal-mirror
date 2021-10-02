package org.firstinspires.ftc.teamcode.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import androidx.annotation.NonNull;

@Config
public class WobbleArm extends Capability {
    private Servo armL, armR, clawL, clawR;
    boolean armState = false, grabberState = false;
    public static double clawOpen = 0.51,
                        clawClosed = 0.73;

    public static double armRelease = 0.37;

    public WobbleArm(Robot parent) {
        super(parent);
    }


    public void init(HardwareMap hardwareMap) {
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        //pin = hardwareMap.get(Servo.class, "grabber");

        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
    }

    public void teleOp(@NonNull Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad2.dpad_up || gamepad1.right_stick_y<-0.5) {
            arm(false);
        } else if (gamepad2.dpad_down || gamepad1.right_stick_y>0.5) {
            arm(true);
        }

        if (gamepad1.right_stick_x>0.5) {//gamepad2.dpad_right ||
            grabber(true);
        } else if (gamepad1.right_stick_x<-0.5) {//gamepad2.dpad_left ||
            grabber(false);
        }
    }

    public void arm(boolean state) {
        arm((state) ? 0.6 : 0.1);
        armState = state;
    }

    public void release() {
        arm(armRelease);
        sleep(650);
        grabber(false);
    }
    public void arm(double pos) {
        armL.setPosition(pos);
        armR.setPosition(1-pos);
    }

    public void grabber(double pos) {
        clawR.setPosition(pos);
        clawL.setPosition(1-pos);
    }

    public void grabber(boolean state) {
        grabber((state) ? clawClosed : clawOpen);
        grabberState = state;
    }
}