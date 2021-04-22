package org.firstinspires.ftc.teamcode.RobotComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.PIDController;
import org.firstinspires.ftc.teamcode.Common.Util;
import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Powers.WheelPowers;
import org.openftc.revextensions2.ExpansionHubMotor;

public class DriveTrain extends Capability {
    private ExpansionHubMotor fRight, bRight, fLeft, bLeft;

    //so bobby asks dad, why did you name my brother sunny? Because I like the sun. Then why did you name me Bobby? Because i lIke BoBs

    public PIDController heading;
    private boolean interrupt = false;
    public WheelPowers powers;

    private boolean fieldCentric = false;

    DriveTrain(Robot parent) {
        super(parent);
    }

    void interruptMovement() {
        interrupt = true;
    }
    public boolean isInterrupted() {
        return interrupt;
    }
    public void resetInterrupt() {
        interrupt = false;
    }

    void init(HardwareMap hardwareMap) {
        fLeft = (ExpansionHubMotor) hardwareMap.get("fLeft");
        bLeft = (ExpansionHubMotor) hardwareMap.get("bLeft");
        fRight = (ExpansionHubMotor) hardwareMap.get("fRight");
        bRight = (ExpansionHubMotor) hardwareMap.get("bRight");

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fLeft.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);

        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        heading = new PIDController(RobotConstants.TRP, RobotConstants.TRI, RobotConstants.TRD);
        heading.setSetpoint(parent.getTargetRotation());
        heading.setInputRange(-180, 180);
        heading.setOutputRange(0, 0.5);
        heading.setAbsoluteTolerance(1);

        powers = new WheelPowers();
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

        double axialPower = -gamepad1.left_stick_y * coefficient;
        double lateralPower = gamepad1.left_stick_x * coefficient;

        double turnPower;
        if(gamepad1.right_trigger > 0) {
            turnPower = -gamepad1.right_trigger;
        } else if (gamepad1.left_trigger > 0) {
            turnPower = gamepad1.left_trigger;
        } else {
            turnPower = 0.0;
        }
        turnPower *= coefficient * parent.getFront();

        VectorD translate;
        if(fieldCentric) {
            translate = new VectorD(axialPower, -lateralPower);
        } else {
            translate = new VectorD(lateralPower, axialPower);
        }

        if(parent.isAiming()) {
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
        }
    }

    public void rotate(int degrees, double power, int timeOutMillis) {
        ElapsedTime timer = new ElapsedTime();

        degrees *= -1;
        PIDController pidRotate = new PIDController(RobotConstants.RP, RobotConstants.RI, RobotConstants.RD);

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);


        pidRotate.reset();
        pidRotate.setSetpoint(degrees + parent.getTargetRotation());
        pidRotate.setInputRange(parent.getTargetRotation(), degrees + parent.getTargetRotation());
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(0.5);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).


        // rotate until turn is completed.
        timer.reset();

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && parent.getPose().getZ() == 0) {
                powers.setTankStrafe(power, -power, 0);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(parent.getPose().getZ()); // power will be - on right turn.
                powers.setTankStrafe(-power, power, 0);
            } while (opModeIsActive() && !pidRotate.onTarget() && timer.milliseconds()<timeOutMillis);
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(parent.getPose().getZ()); // power will be + on left turn.
                powers.setTankStrafe(-power, power, 0);
            } while (opModeIsActive() && !pidRotate.onTarget() && timer.milliseconds()<timeOutMillis);

        powers.set(0);

        parent.setTargetRotation(degrees + parent.getTargetRotation());
        sleep(200);
    }


    @Override
    public void run() {
        heading.enable();
        while(opModeIsActive() && !isInterrupted()) {
            if(parent.isAiming()) {
                VectorD vector = Util.clipToXY(parent.aimPos).subtracted(parent.getPosition());
                double ang = Math.toDegrees(Math.atan2(vector.getY(), vector.getX())) - 90 +
                        RobotConstants.SHOOTING_OFFSET_ANGLE + (Math.round(parent.getPose().getZ()/360) * 360);
                heading.setSetpoint(ang);
                powers.setRotation(heading.performPID(parent.getPose().getZ()));
            }
            setPowers();
        }
    }
}
