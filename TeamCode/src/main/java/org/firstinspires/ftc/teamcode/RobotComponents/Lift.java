package org.firstinspires.ftc.teamcode.RobotComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotComponents.Tasks.ExtentPowerTask;
import org.firstinspires.ftc.teamcode.RobotComponents.Tasks.Task;
import org.firstinspires.ftc.teamcode.RobotComponents.Tasks.TwoStateTask;
import org.firstinspires.ftc.teamcode.Common.doubleDCMotor;

import java.util.HashMap;
import java.util.Map;

public class Lift extends Capability {
    public doubleDCMotor winch;
    public DigitalChannel mag;

    public Servo grab;
    public Servo park;

    public Servo depo1;
    public Servo depo2;


    boolean g2LBReleased = true;
    boolean g2RBReleased = true;

    int teleopLiftLevel = 1;

    double autoTargetLevel = 0;

    private int currLiftTaskState = 0;
    private int prevLiftTaskState = -1;
    private Robot.TaskState prevRobotTaskState = Robot.TaskState.IDLE;


    public Lift(Robot parentRobot) {
        super(parentRobot);
        winch = new doubleDCMotor(parent, true, false);
    }

    public RaiseLiftTask RESET_LIFT = new RaiseLiftTask(0, 1);
    public RaiseLiftTask RAISE_LIFT = new RaiseLiftTask(700, 1);
    public GrabberTask GRAB = new GrabberTask(true);
    public GrabberTask RELEASE = new GrabberTask(false);

    public enum TASKS {
        RESET_LIFT, RAISE_LIFT, GRAB, RELEASE
    }

    HashMap<TASKS, Task> enumTaskMap = new HashMap<TASKS, Task>() {
        {
            put(TASKS.RESET_LIFT, RESET_LIFT);
            put(TASKS.RAISE_LIFT, RAISE_LIFT);
            put(TASKS.GRAB, GRAB);
            put(TASKS.RELEASE, RELEASE);
        }
    };

    public RaiseLiftTask createCustomLiftTask(double height, double power) {
        return new RaiseLiftTask(height, power);
    }

    private HashMap<Double, Task> mapTemplateToObjects(HashMap<Double, TASKS> map) {
        HashMap<Double, Task> newMap = new HashMap<>();

        for(Map.Entry element : map.entrySet()) {
            newMap.put((Double)element.getKey(), enumTaskMap.get((TASKS)element.getValue()));
        }
        return newMap;
    }


    HashMap<Robot.TaskState, Double> liftTaskTimes = new HashMap<Robot.TaskState, Double>() {
        {
            put(Robot.TaskState.GO_TO_FOUNDATION, 0.55);
            put(Robot.TaskState.GO_BACK_TO_STONE, 0.1);
            put(Robot.TaskState.PARK, null);
            put(Robot.TaskState.GRAB_FOUNDATION, 0.1);


            put(Robot.TaskState.GO_TO_STONE, null);
            put(Robot.TaskState.IDLE, null);
            put(Robot.TaskState.GRAB_STONE, null);
            put(Robot.TaskState.DROP_STONE, null);
            put(Robot.TaskState.MOVE_FOUNDATION, null);
        }
    };

    HashMap<Robot.TaskState, Integer> liftTaskHeight = new HashMap<Robot.TaskState, Integer>() {
        {
            put(Robot.TaskState.GO_TO_FOUNDATION, 600);
            put(Robot.TaskState.GO_BACK_TO_STONE, 0);
            put(Robot.TaskState.PARK, null);
            put(Robot.TaskState.GRAB_FOUNDATION, 500);


            put(Robot.TaskState.GO_TO_STONE, null);
            put(Robot.TaskState.IDLE, null);
            put(Robot.TaskState.GRAB_STONE, null);
            put(Robot.TaskState.DROP_STONE, null);
            put(Robot.TaskState.MOVE_FOUNDATION, null);
        }
    };
    HashMap<Robot.TaskState, Double> grabberTaskTimes = new HashMap<Robot.TaskState, Double>() {
        {
            put(Robot.TaskState.GO_TO_FOUNDATION, null);
            put(Robot.TaskState.GO_BACK_TO_STONE, 0.7);
            put(Robot.TaskState.PARK, null);
            put(Robot.TaskState.GRAB_FOUNDATION, null);
            put(Robot.TaskState.GO_TO_STONE, null);
            put(Robot.TaskState.IDLE, null);
            put(Robot.TaskState.GRAB_STONE, null);
            put(Robot.TaskState.DROP_STONE, null);
            put(Robot.TaskState.MOVE_FOUNDATION, null);
        }
    };

    HashMap<Robot.TaskState, Boolean> grabberTaskState = new HashMap<Robot.TaskState, Boolean>() {
        {
            put(Robot.TaskState.GO_TO_FOUNDATION, null);
            put(Robot.TaskState.GO_BACK_TO_STONE, false);
            put(Robot.TaskState.PARK, null);
            put(Robot.TaskState.GRAB_FOUNDATION, null);
            put(Robot.TaskState.GO_TO_STONE, null);
            put(Robot.TaskState.IDLE, null);
            put(Robot.TaskState.GRAB_STONE, null);
            put(Robot.TaskState.DROP_STONE, null);
            put(Robot.TaskState.MOVE_FOUNDATION, null);
        }
    };

    private Robot.TaskState currentLiftState = Robot.TaskState.IDLE;
    private Robot.TaskState currentGrabberState = Robot.TaskState.IDLE;
    @Override
    public void run() {


        while (opModeIsActive()) {
            Robot.TaskState ts = parent.getTaskState();

            if(liftTaskTimes.get(ts) != null && ts != currentLiftState) {
                while ( (parent.driveTrain.getProgress() < liftTaskTimes.get(ts)) && opModeIsActive()) {
                    sleep(1);
                }
                int height = liftTaskHeight.get(ts);
                if (height == 0) {
                    grabber(true);
                    resetLift(1, 1000);
                } else {
                    raiseLift(height, 1);
                }
            }
            currentLiftState = ts;


            if(grabberTaskTimes.get(ts) != null && ts != currentGrabberState) {
                while ( (parent.driveTrain.getProgress() < grabberTaskTimes.get(ts)) && opModeIsActive()) {
                    sleep(1);
                }
                boolean state = grabberTaskState.get(ts);
                grabber(state);
            }
            currentGrabberState = ts;
        }
    }

    void init() {
        winch.init(parent.getMyOpMode().hardwareMap.get(DcMotor.class, "lift1"),
                parent.getMyOpMode().hardwareMap.get(DcMotor.class, "lift2"));
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mag = parent.getMyOpMode().hardwareMap.get(DigitalChannel.class, "liftMag");

        depo1 = parent.getMyOpMode().hardwareMap.get(Servo.class, "depo1");
        depo2 = parent.getMyOpMode().hardwareMap.get(Servo.class, "depo2");

        depo1.setPosition(0);
        depo2.setPosition(1);

        grab = parent.getMyOpMode().hardwareMap.get(Servo.class, "grab");
        park = parent.getMyOpMode().hardwareMap.get(Servo.class, "park");

        park.setPosition(0.85);
    }

    public void increaseLiftHeight(int millis) {
        int old = liftTaskHeight.get(Robot.TaskState.GO_TO_FOUNDATION);
        liftTaskHeight.remove(Robot.TaskState.GO_TO_FOUNDATION);
        liftTaskHeight.put(Robot.TaskState.GO_TO_FOUNDATION, old + millis);
    }

    public void setTarget(double target) {
        autoTargetLevel = target;
    }



    public void grabber(Boolean state) {
        if(state) {
            grab.setPosition(0);
        } else {
            grab.setPosition(0.4);
        }
        sleep(250);
    }

    public void resetLift(double pow, int timeOutMillis) {
        ElapsedTime timer = new ElapsedTime();
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winch.setPower(-Math.abs(pow));
        timer.reset();
        while(!parent.sensors.getLiftSwitchState() && !isStopRequested() && timer.milliseconds() < timeOutMillis) {
            sleep(1);
        }
        winch.setPower(0);
    }

    public void goToInches(double inches, double pow) {  //diameter 30 mm      circ  3.711 in
        double ang = inches / 3.711;
        int clicks = (int)(ang * 288);
        winch.setTargetPosition(winch.getCurrentPosition()+clicks);

        winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(winch.isBusy() & !isStopRequested()){
            winch.setPower(-pow);
            parent.getMyOpMode().telemetry.addData("target", winch.getTargetPosition());
            parent.getMyOpMode().telemetry.addData("current pos", winch.getCurrentPosition());
            parent.getMyOpMode().telemetry.addData("power", winch.getPower());
            parent.getMyOpMode().telemetry.update();
        }
        winch.setPower(0);
        parent.getMyOpMode().telemetry.addLine("ok done");
        parent.getMyOpMode().telemetry.update();
    }

    public void raiseLift(int millis, double pow) {
        winch.setPower(pow);
        sleep(millis);
        winch.setPower(0);
    }

    public void goToLevel(int level, double pow) {
        double inches = ((level-1) * 5.0 + 3);
        goToInches(inches, pow);
    }



    public void teleOp(Gamepad gamepad2) {
        double liftPow = 0;

        if(!winch.isBusy()) {
            winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            liftPow = -gamepad2.right_stick_y;


            if (parent.sensors.getLiftSwitchState() && liftPow < 0) {
                liftPow = 0;
            }

            winch.setPower(liftPow);
        }

        if(gamepad2.right_bumper && g2RBReleased && !winch.isBusy()) {
            double inches = 4 * teleopLiftLevel;

            double ang = inches / (1.5/2.54);
            int clicks = (int)(ang/(2*Math.PI) * 288);

            winch.setTargetPosition(winch.getCurrentPosition() + clicks);
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            winch.setPower(1);

            teleopLiftLevel++;
            g2RBReleased = false;
        }
        if(!gamepad2.right_bumper) {
            g2RBReleased = true;
        }

        if(gamepad2.left_bumper && g2LBReleased) {
            if(teleopLiftLevel != 1) {
                teleopLiftLevel--;}
            g2LBReleased = false;
        }
        if(!gamepad2.left_bumper) {
            g2LBReleased = true;
        }

        //double ratio = speed/winch.getPower();
        //telemetry.addData("ratio:       ", ratio);
        // telemetry.addData("disableLift: ", disableLift);

        /*if((Math.abs(ratio) < 1.5 && !Double.isInfinite(ratio)) || disableLift) {
            liftPow = 0;
            disableLift = true;
            speed = 0;
        }
        if(gamepad2.y) {
            disableLift = false;
        }*/

        if(gamepad2.a) {
            park.setPosition(1);
        } else if( gamepad2.b) {
            park.setPosition(0);
        }



        //COLLECTOR
        if(gamepad2.dpad_down) {
            grabber(true);
        } else if(gamepad2.dpad_up) {
            grabber(false);
        } else if(gamepad2.dpad_right) {
            grab.setPosition(0.4);
        }

        //MARKER DEPOSIT
        if(gamepad2.y) {
            depo1.setPosition(0);
            depo2.setPosition(1);
        } else if(gamepad2.x) {
            depo1.setPosition(1);
            depo2.setPosition(0);
        }

    }



    public class RaiseLiftTask extends ExtentPowerTask {
        RaiseLiftTask(double heightMillis, double power) {
            super(heightMillis, power);
        }

        @Override
        public void run() {
            if(extent == 0) {
                resetLift(power, 2000);
            } else {
                raiseLift((int)extent, power);
            }
        }
    }

    public class GrabberTask extends TwoStateTask {
        public GrabberTask(boolean state) {
            super(state);
        }

        @Override
        public void run() {
            grabber(state);
        }
    }

}
