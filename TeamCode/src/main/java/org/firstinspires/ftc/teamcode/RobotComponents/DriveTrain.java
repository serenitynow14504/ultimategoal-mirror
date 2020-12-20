package org.firstinspires.ftc.teamcode.RobotComponents;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.PIDController;
import org.firstinspires.ftc.teamcode.Common.Utilities;
import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Powers.WheelPowers;
import org.openftc.revextensions2.ExpansionHubMotor;

import androidx.annotation.NonNull;

public class DriveTrain extends Capability {
    private ExpansionHubMotor fRight, bRight, fLeft, bLeft;

    //so bobby asks dad, why did you name my brother sunny? Because I like the sun. Then why did you name me Bobby? Because i lIke BoBs

    //PIDController x = new PIDController(0.00015,0.0000004,0.0005);//d = 0.0004
    //PIDController y = new PIDController(0.0001,0.00000015,0.0005);
    //PIDController r = new PIDController(0.015,0,0.007);//p = 0.01   d = 0.006

    PIDController heading;
    private boolean interrupt = false;
    public WheelPowers powers;

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
        heading.setOutputRange(0, 0.5);  //-power to power?

        powers = new WheelPowers();
    }

    @Deprecated
    double[] scalePowers(double[] powers) {
        double[] newPowers = powers;

        double max = 0;
        for(int i = 0; i<4; i++) {
            if(newPowers[i] > max) max = newPowers[i];
        }

        if(max > 1) {
            for(int i = 0; i<4; i++) {
                newPowers[i] /= max;
            }
        }

        return newPowers;
    }


    @Deprecated
    public void setScaledPowersFromGlobalVector(@NonNull VectorD xy, double r) {
        double robotRot = -Math.toRadians(parent.getPosition().getZ());
        VectorD localVector = new VectorD(
                (xy.getX() * Math.cos(robotRot) - xy.getY() * Math.sin(robotRot)),
                (xy.getX() * Math.sin(robotRot) + xy.getY() * Math.cos(robotRot))
        );

        setScaledPowersFromComponents(localVector.getX(), localVector.getY(), r);

    }
    @Deprecated
    public void setScaledPowersFromGlobalVector(@NonNull Pair<VectorD, Double> p) {
        double robotRot = Math.toRadians(parent.getPosition().getZ());
        VectorD localVector = new VectorD(
                (p.first.getX() * Math.cos(robotRot) - p.first.getY() * Math.sin(robotRot)),
                (p.first.getX() * Math.sin(robotRot) + p.first.getY() * Math.cos(robotRot))
        );

        setScaledPowersFromComponents(localVector.getX(), localVector.get(1), p.second);

    }
    @Deprecated
    public void setScaledPowersFromComponents(double xPower, double yPower, double rPower) {
        double[] powers = new double[] {
                yPower + xPower - rPower,
                yPower - xPower + rPower,
                yPower - xPower - rPower,
                yPower + xPower + rPower
        };
        setPowers(scalePowers(powers));
    }
    @Deprecated
    public void setPowersFromComponents(double xPower, double yPower, double rPower) {
        double[] powers = new double[] {
                yPower + xPower - rPower,
                yPower - xPower + rPower,
                yPower - xPower - rPower,
                yPower + xPower + rPower
        };
        setPowers(powers);
    }
    @Deprecated
    public void setScaledTankStrafePowers(double l, double r, double strafe) {
        double[] powers = new double[] {
                l + strafe,
                r - strafe,
                l - strafe,
                r + strafe
        };
        setPowers(scalePowers(powers));
    }
    @Deprecated
    public void setPowers(double speed) {
        fLeft.setPower(speed);
        fRight.setPower(speed);
        bLeft.setPower(speed);
        bRight.setPower(speed);
    }
    @Deprecated
    public void setPowers(double fl, double fr, double bl, double br) {
        fLeft.setPower(fl);
        fRight.setPower(fr);
        bLeft.setPower(bl);
        bRight.setPower(br);
    }
    @Deprecated
    public void setPowers(double[] powers) {
        fLeft.setPower(powers[0]);
        fRight.setPower(powers[1]);
        bLeft.setPower(powers[2]);
        bRight.setPower(powers[3]);
    }
    @Deprecated
    public void setPowers(double l, double r) { //for tank drive
        fLeft.setPower(l);
        fRight.setPower(r);
        bLeft.setPower(l);
        bRight.setPower(r);
    }

    void setPowers(WheelPowers wp) {
        fLeft.setPower(wp.fl);
        fRight.setPower(wp.fr);
        bLeft.setPower(wp.bl);
        bRight.setPower(wp.br);
    }


    void teleOp(Gamepad gamepad1, Gamepad gamepad2) {
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
            turnPower = gamepad1.right_trigger;
        } else if (gamepad1.left_trigger > 0) {
            turnPower = -gamepad1.left_trigger;
        } else {
            turnPower = 0.0;
        }
        turnPower *= coefficient * parent.getFront();

        if(parent.autoAim) {
            powers.setLocalTrans(new VectorD(lateralPower, axialPower));
        } else {
            powers.setFromLocalVector(new VectorD(lateralPower, axialPower, turnPower));
        }

        /*double fl = (axialPower + lateralPower);
        double fr = (axialPower - lateralPower);
        double bl = (axialPower - lateralPower);
        double br = (axialPower + lateralPower);


        fl += turnPower;
        fr -= turnPower;
        bl += turnPower;
        br -= turnPower;

        fLeft.setPower(parent.getFront() * fl);
        fRight.setPower(parent.getFront() * fr);
        bLeft.setPower(parent.getFront() * bl);
        bRight.setPower(parent.getFront() * br);*/
    }

    public void rotate(int degrees, double power, int timeOutMillis) {
        ElapsedTime timer = new ElapsedTime();

        degrees *= -1;
        PIDController pidRotate = new PIDController(RobotConstants.RP, RobotConstants.RI, RobotConstants.RD);
        // restart imu angle tracking.
        //parent.imu.resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

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
            while (opModeIsActive() && parent.imu.getAngle() == 0) {
                setPowers(power, -power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(parent.imu.getAngle()); // power will be - on right turn.
                setPowers(-power, power);
            } while (opModeIsActive() && !pidRotate.onTarget() && timer.milliseconds()<timeOutMillis);
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(parent.imu.getAngle()); // power will be + on left turn.
                setPowers(-power, power);
            } while (opModeIsActive() && !pidRotate.onTarget() && timer.milliseconds()<timeOutMillis);

        setPowers(0);

        //rotation = getAngle();

        // wait for rotation to stop.
        //parent.getMyOpMode().telemetry.addData("target", parent.getTargetRotation());
        //parent.getMyOpMode().telemetry.update();

        parent.setTargetRotation(degrees + parent.getTargetRotation());
        sleep(200);
    }


    @Override
    public void run() {
        heading.enable();
        while(opModeIsActive() && !isInterrupted()) {
            if(parent.autoAim) {
                VectorD vector = Utilities.clipToXY(parent.aimPos).subtracted(parent.get2DPosition());
                double ang = Math.toDegrees(Math.atan2(vector.getY(), vector.getX())) - 90 + RobotConstants.SHOOTING_OFFSET_ANGLE;
                heading.setSetpoint(ang);
                powers.setRotation(heading.performPID(parent.getPosition().getZ()));
            }
            setPowers(powers);
        }
    }
}
