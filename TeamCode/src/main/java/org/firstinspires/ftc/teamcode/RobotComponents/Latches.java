package org.firstinspires.ftc.teamcode.RobotComponents;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotComponents.Tasks.TwoStateTask;

public class Latches extends Capability {
    private Servo left, right, fleft, fright;




    public Latches(Robot parent) {
        super(parent);
    }

    public LatchTask LATCH = new LatchTask(true);
    public LatchTask UNLATCH = new LatchTask(false);



    void init() {
        left = parent.getMyOpMode().hardwareMap.get(Servo.class, "hookL");
        right = parent.getMyOpMode().hardwareMap.get(Servo.class, "hookR");
        fleft = parent.getMyOpMode().hardwareMap.get(Servo.class, "fookL");
        fright = parent.getMyOpMode().hardwareMap.get(Servo.class, "fookR");

    }

    public void latch(Boolean state) {
        if(state) {
            right.setPosition(0);
            left.setPosition(1);
            fright.setPosition(0);
            fleft.setPosition(1);
        } else {
            right.setPosition(1);
            left.setPosition(0);
            fright.setPosition(1);
            fleft.setPosition(0);
        }
    }

    private boolean done = false;

    @Override
    public void run(){
        while(opModeIsActive()) {
            if(parent.getTaskState() == Robot.TaskState.GRAB_FOUNDATION && parent.sensors.getFrontBumperState() && !done) {
                sleep(100);
                latch(true);
                done = true;
                if(parent.getTaskState() == Robot.TaskState.GRAB_FOUNDATION && parent.sensors.getFrontBumperState() && !done) {
                    parent.driveTrain.interruptMovement();
                }
            }
        }
    }

    public void teleOp(Gamepad gamepad1) {
        if(gamepad1.y) {
            latch(true);
        } else if (gamepad1.x) {
            latch(false);
        }
    }



    public class LatchTask extends TwoStateTask {
        public LatchTask(boolean state) {
            super(state);
        }

        @Override
        public void run() {
            latch(state);
        }
    }
}
