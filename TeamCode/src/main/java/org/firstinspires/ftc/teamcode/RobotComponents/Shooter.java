package org.firstinspires.ftc.teamcode.RobotComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

class Shooter extends Capability {
    private ExpansionHubMotor flyWheel, platform;
    private Servo pusherL, pusherR;
    private DistanceSensor platformSensor;
    private double power = 0;
    private final double CURRENT_THRESHOLD = 10;
    private final int POSITION_DIFFERENCE = 8924, liftPosThresh = 60;
    private int bottomPos, topPos, targetLiftPos;

    public Shooter(Robot parent) {
        super(parent);
    }

    void init(HardwareMap hardwareMap) {
        flyWheel = (ExpansionHubMotor) hardwareMap.get("encY");
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        platform = (ExpansionHubMotor)hardwareMap.get("platform");
        platform.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pusherL = hardwareMap.get(Servo.class, "pusherL");
        pusherR = hardwareMap.get(Servo.class, "pusherR");
        platformSensor = hardwareMap.get(DistanceSensor.class, "platformSensor");
        topPos = platform.getCurrentPosition();
        bottomPos = topPos - POSITION_DIFFERENCE;
        targetLiftPos = topPos;

    }

    public void setPower(double pow) {
        power = pow;
    }

    public void teleOp(Gamepad gamepad) {
        /*if(gamepad.dpad_up) {
            setPower(-1);
        } else if(gamepad.dpad_down) {
            setPower(0);
        }*/
        setPower(-1);//-1

        if(gamepad.right_bumper) {
            pusherL.setPosition(0.7);
            pusherR.setPosition(0.5);
            targetLiftPos = topPos;
        } else {
            pusherL.setPosition(1);
            pusherR.setPosition(0);
        }
        /*telemetry.addData("lift pos: ", platform.getCurrentPosition());
        telemetry.update();*/
        //platform.setPower(gamepad.right_stick_y/3);
        if(gamepad.right_stick_button && gamepad.right_stick_y<-0.5) {
            //platform.setTargetPosition(topPos);
            //platform.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //platform.setPower(0.2);
            targetLiftPos = topPos;
        }
        if(gamepad.right_stick_button && gamepad.right_stick_y>0.5) {
            //platform.setTargetPosition(bottomPos);
            //platform.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //platform.setPower(-0.2);
            targetLiftPos = bottomPos;
        }
        if(gamepad.dpad_right) {
            targetLiftPos = bottomPos;
        }
    }

    boolean platformFull() {
        return platformSensor.getDistance(DistanceUnit.CM) < 5;
    }

    void platform(boolean state) {
        if(state) {
            platform.setTargetPosition(0);
        } else {
            platform.setTargetPosition(0);
        }
    }

    void push() {
        pusherL.setPosition(1);
        pusherR.setPosition(1);
        pusherL.setPosition(0);
        pusherR.setPosition(0);
    }

    public void fire() {
        platform(true);
        for(int i = 0; i < 3 && platformFull(); i++) {
            push();
        }
        platform(false);
    }

    @Override
    public void run() {
        while(opModeIsActive()) {
            double newPow = flyWheel.getPower();
            if(newPow < power) {
                newPow += 0.02;
            } else if(newPow > power) {
                newPow -= 0.02;
            }
            flyWheel.setPower(newPow);
            if (flyWheel.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > CURRENT_THRESHOLD) {
                setPower(0);
                parent.setLedColors(255, 0, 0);
            }

            if(platform.getCurrentPosition() < targetLiftPos - liftPosThresh) {
                platform.setPower(0.2);
            } else if (platform.getCurrentPosition() > targetLiftPos + liftPosThresh) {
                platform.setPower(-0.2);
            } else {
                platform.setPower(0);
            }
        }
    }
}
