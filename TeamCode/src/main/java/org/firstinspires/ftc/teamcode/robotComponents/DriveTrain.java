package org.firstinspires.ftc.teamcode.robotComponents;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.common.PIDController;
import org.firstinspires.ftc.teamcode.common.VectorD;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robotComponents.MovementControllers.MovementController;
import org.firstinspires.ftc.teamcode.robotComponents.MovementControllers.VectorController;
import org.firstinspires.ftc.teamcode.robotComponents.Powers.WheelPowers;

import static org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants.WHEELBASE;
import static org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants.WHEEL_DIAMETER_INCHES;

public class DriveTrain extends Capability {
    //private ExpansionHubMotor fRight, bRight, fLeft, bLeft;

    private DTWheel fRight, bRight, fLeft, bLeft;
    private RealMatrix forwardMatrix = new Array2DRowRealMatrix(new double[][] {
            { 1, 1, -(TRACK_WIDTH+WHEELBASE) },
            { 1, -1, -(TRACK_WIDTH+WHEELBASE) },
            { 1, 1, (TRACK_WIDTH+WHEELBASE) },
            { 1, -1, (TRACK_WIDTH+WHEELBASE) }
            }, false);

    public MovementController controller;

    //so bobby asks dad, why did you name my brother sunny? Because I like the sun. Then why did you name me Bobby? Because i lIke BoBs

    public PIDController heading;
    private boolean interrupt = false;
    public WheelPowers powers;

    private boolean fieldCentric = false;

    DriveTrain(Robot parent) {
        super(parent);
        controller = new VectorController(parent);
    }

    void interruptMovement() {
        interrupt = true;
    }
    public boolean isDtInterrupted() {
        return interrupt;
    }
    public void resetInterrupt() {
        interrupt = false;
    }

    void init(HardwareMap hardwareMap) {
        fLeft = new DTWheel(robot, "fLeft", false);
        bLeft = new DTWheel(robot, "bLeft", false);
        fRight = new DTWheel(robot, "fRight", true);
        bRight = new DTWheel(robot, "bRight", true);

        fLeft.init(hardwareMap);
        bLeft.init(hardwareMap);
        fRight.init(hardwareMap);
        bRight.init(hardwareMap);




//        fLeft = (ExpansionHubMotor) hardwareMap.get("fLeft");
//        bLeft = (ExpansionHubMotor) hardwareMap.get("bLeft");
//        fRight = (ExpansionHubMotor) hardwareMap.get("fRight");
//        bRight = (ExpansionHubMotor) hardwareMap.get("bRight");
//
//        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        fLeft.setDirection(DcMotor.Direction.FORWARD);
//        bLeft.setDirection(DcMotor.Direction.FORWARD);
//        fRight.setDirection(DcMotor.Direction.REVERSE);
//        bRight.setDirection(DcMotor.Direction.REVERSE);
//
//        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        heading = new PIDController(RobotConstants.TRP, RobotConstants.TRI, RobotConstants.TRD);
        heading.setSetpoint(robot.getTargetRotation());
        heading.setInputRange(-180, 180);
        heading.setOutputRange(0, 0.5);
        heading.setAbsoluteTolerance(1);

        powers = new WheelPowers();
    }

    public void setLocalTransl(VectorD v) {
        double[] fin = forwardMatrix.operate(new double[] {v.getX(), v.getY(), v.getZ()});
        for(int i = 0; i<fin.length; i++)
        {
            fin[i] /= WHEEL_DIAMETER_INCHES/4;//2
        }
        fLeft.setVelocity(fin[0]);
        bLeft.setVelocity(fin[1]);
        bRight.setVelocity(fin[2]);
        fRight.setVelocity(fin[3]);
    }

    public void setPowers() {
        fLeft.setPower(powers.getFL());
        fRight.setPower(powers.getFR());
        bLeft.setPower(powers.getBL());
        bRight.setPower(powers.getBR());
    }

    void teleOp(Gamepad gamepad1, Gamepad gamepad2) {
        //RobotConstants.SHOOTING_OFFSET_ANGLE = 13.5;
        if(gamepad1.left_bumper) {
            fieldCentric = false;
        } else if(gamepad1.right_bumper) {
            fieldCentric = true;
        }
        double coefficient = 0.525;
        if(gamepad1.a) {
            coefficient = 0.3;
        } else if(gamepad1.b) {
            coefficient = 1;
        }

        double lateralPower = -gamepad1.left_stick_y * DTWheel.MAXIMUM_MAXIMUM_VELOCITY;
        double axialPower = gamepad1.left_stick_x * DTWheel.MAXIMUM_MAXIMUM_VELOCITY;

        double turnPower;
        if(gamepad1.right_trigger > 0) {
            turnPower = -gamepad1.right_trigger;
        } else if (gamepad1.left_trigger > 0) {
            turnPower = gamepad1.left_trigger;
        } else {
            turnPower = 0.0;
        }
        turnPower *= coefficient;

        VectorD translate;
        if(fieldCentric) {
            translate = new VectorD(axialPower, -lateralPower, 0);
        } else {
            translate = new VectorD(lateralPower, axialPower, 0);
        }

        setLocalTransl(translate);
        /*if(parent.isAiming()) {
            if(fieldCentric) {
                powers.setGlobalTrans(translate, parent.getPose().getZ());
            } else {
                powers.setLocalTrans(translate);
            }
        } else {
            if(fieldCentric) {
                powers.setFromGlobalVector(Util.addZ(translate, turnPower), parent.getPose().getZ());
            } else {
                powers.setFromLocalVector(Util.addZ(translate, turnPower));
            }
        }*/
    }

    public void rotate(int degrees, double power, int timeOutMillis) {
        ElapsedTime timer = new ElapsedTime();

        degrees *= -1;
        PIDController pidRotate = new PIDController(RobotConstants.RP, RobotConstants.RI, RobotConstants.RD);

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);


        pidRotate.reset();
        pidRotate.setSetpoint(degrees + robot.getTargetRotation());
        pidRotate.setInputRange(robot.getTargetRotation(), degrees + robot.getTargetRotation());
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(0.5);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).


        // rotate until turn is completed.
        timer.reset();

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && robot.getPose().getZ() == 0) {
                powers.setTankStrafe(power, -power, 0);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(robot.getPose().getZ()); // power will be - on right turn.
                powers.setTankStrafe(-power, power, 0);
            } while (opModeIsActive() && !pidRotate.onTarget() && timer.milliseconds()<timeOutMillis);
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(robot.getPose().getZ()); // power will be + on left turn.
                powers.setTankStrafe(-power, power, 0);
            } while (opModeIsActive() && !pidRotate.onTarget() && timer.milliseconds()<timeOutMillis);

        powers.set(0);

        robot.setTargetRotation(degrees + robot.getTargetRotation());
        sleep(200);
    }


    @Override
    public void run() {
//        fLeft.start();
//        fRight.start();
//        bLeft.start();
//        bRight.start();
//        heading.enable();
        while(opModeIsActive() && !isDtInterrupted()) {
            /*if(parent.isAiming()) {
                VectorD vector = Util.clipToXY(parent.aimPos).subtracted(parent.getPosition());
                double ang = Math.toDegrees(Math.atan2(vector.getY(), vector.getX())) - 90 +
                        RobotConstants.SHOOTING_OFFSET_ANGLE + (Math.round(parent.getPose().getZ()/360) * 360);
                heading.setSetpoint(ang);
                powers.setRotation(heading.performPID(parent.getPose().getZ()));
            }*/
            //Util.log("dt loop");
            setPowers();

        }
    }

    @Override
    void tick(TelemetryPacket packet) {
        controller.tick(robot.getPose(), packet);

        setPowers();

//        fLeft.tick(packet);
//        fRight.tick(packet);
//        bLeft.tick(packet);
//        bRight.tick(packet);
    }
}
