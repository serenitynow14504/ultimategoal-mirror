package org.firstinspires.ftc.teamcode.RobotComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotComponents.Tasks.ExtentPowerTask;

public class Tape extends Capability {
    private DcMotor wheel;

    public Tape(Robot parent) {
        super(parent);
    }


    void init(){
        wheel = parent.getMyOpMode().hardwareMap.get(DcMotor.class, "encY");
        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void teleOp(Gamepad gamepad2) {
        wheel.setPower(-gamepad2.left_stick_y);
    }


    private boolean done = false;

    public void setPower(double pow) {
        wheel.setPower(-pow);
    }

    @Override
    public void run() {
        while(opModeIsActive()) {
            if(parent.getTaskState() == Robot.TaskState.PARK && parent.driveTrain.getProgress() > 0.8 && opModeIsActive() && !done) {
                ElapsedTime timer = new ElapsedTime();

                while (timer.milliseconds() < 2000 && opModeIsActive()) {
                    wheel.setPower(1);
                }
                wheel.setPower(0);
                done = true;
            }
        }
    }



    public class TapeTask extends ExtentPowerTask {
        public TapeTask(double timeMillis, double power){
            super(timeMillis, power);
        }

        @Override
        public void run() {
            ElapsedTime timer = new ElapsedTime();
            while(timer.milliseconds() < extent) {
                setPower(power);
            }
            setPower(0);
        }
    }
}
