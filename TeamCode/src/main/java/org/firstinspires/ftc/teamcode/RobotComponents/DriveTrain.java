package org.firstinspires.ftc.teamcode.RobotComponents;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.PIDController;
import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.openftc.revextensions2.ExpansionHubMotor;

import androidx.annotation.NonNull;

public class DriveTrain extends Capability {
    private ExpansionHubMotor right, right2, left, left2;

    //so bobby asks dad, why did you name my brother sunny? Because I like the sun. Then why did you name me Bobby? Because i lIke BoBs

    //PIDController x = new PIDController(0.00015,0.0000004,0.0005);//d = 0.0004
    //PIDController y = new PIDController(0.0001,0.00000015,0.0005);
    //PIDController r = new PIDController(0.015,0,0.007);//p = 0.01   d = 0.006

    private boolean interrupt = false;

    private double botRot=0;

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
        left = (ExpansionHubMotor) hardwareMap.get("fLeft");
        left2 = (ExpansionHubMotor) hardwareMap.get("bLeft");
        right = (ExpansionHubMotor) hardwareMap.get("fRight");
        right2 = (ExpansionHubMotor) hardwareMap.get("bRight");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    double[] scalePowers(double fl, double fr, double bl, double br) {
        double[] newPowers = new double[] {
                fl, fr, bl, br
        };

        double max = 0;
        if(fl > max) max = fl;
        if(fr > max) max = fr;
        if(bl > max) max = bl;
        if(br > max) max = br;

        if(max > 1) {
            for(int i = 0; i<4; i++) {
                newPowers[i] /= max;
            }
        }

        return newPowers;
    }

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

    public void setScaledPowersFromGlobalVector(@NonNull VectorD xy, double r) {
        double robotRot = Math.toRadians(parent.getPosition().getR());
        VectorD localVector = new VectorD(
                (float)(xy.getX() * Math.cos(robotRot) - xy.getY() * Math.sin(robotRot)),
                (float)(xy.getX() * Math.sin(robotRot) + xy.getY() * Math.cos(robotRot))
        );

        setScaledPowersFromComponents(localVector.getX(), localVector.getY(), r);

    }

    public void setScaledPowersFromGlobalVector(@NonNull Pair<VectorD, Double> p) {
        double robotRot = Math.toRadians(parent.getPosition().getR());
        VectorD localVector = new VectorD(
                (float)(p.first.getX() * Math.cos(robotRot) - p.first.getY() * Math.sin(robotRot)),
                (float)(p.first.getX() * Math.sin(robotRot) + p.first.getY() * Math.cos(robotRot))
        );

        setScaledPowersFromComponents(localVector.getX(), localVector.get(1), p.second);

    }

    public void setScaledPowersFromComponents(double xPower, double yPower, double rPower) {
        double[] powers = new double[] {
                yPower + xPower - rPower,
                yPower - xPower + rPower,
                yPower - xPower - rPower,
                yPower + xPower + rPower
        };
        setPowers(scalePowers(powers));
    }

    public void setPowersFromComponents(double xPower, double yPower, double rPower) {
        double[] powers = new double[] {
                yPower + xPower - rPower,
                yPower - xPower + rPower,
                yPower - xPower - rPower,
                yPower + xPower + rPower
        };
        setPowers(powers);
    }

    public void setScaledTankStrafePowers(double l, double r, double strafe) {
        double[] powers = new double[] {
                l + strafe,
                r - strafe,
                l - strafe,
                r + strafe
        };
        setPowers(scalePowers(powers));
    }

    public void setPowers(double speed) {
        left.setPower(speed);
        right.setPower(speed);
        left2.setPower(speed);
        right2.setPower(speed);
    }

    public void setPowers(double fl, double fr, double bl, double br) {
        left.setPower(fl);
        right.setPower(fr);
        left2.setPower(bl);
        right2.setPower(br);
    }

    public void setPowers(double[] powers) {
        left.setPower(powers[0]);
        right.setPower(powers[1]);
        left2.setPower(powers[2]);
        right2.setPower(powers[3]);
    }

    public void setPowers(double l, double r) { //for tank drive
        left.setPower(l);
        right.setPower(r);
        left2.setPower(l);
        right2.setPower(r);
    }


    void teleOp(Gamepad gamepad) {
        double coefficient = 0.525;
        if(gamepad.a) {
            coefficient = 0.3;
        } else if(gamepad.b) {
            coefficient = 1;
        }

        double axialPower = -gamepad.left_stick_y * coefficient;
        double lateralPower = gamepad.left_stick_x * coefficient;

        double turnPower;
        if(gamepad.right_trigger > 0) {
            turnPower = gamepad.right_trigger;
        } else if (gamepad.left_trigger > 0) {
            turnPower = -gamepad.left_trigger;
        } else {
            turnPower = 0.0;
        }
        turnPower *= coefficient * parent.getFront();

        double fl = (axialPower + lateralPower);
        double fr = (axialPower - lateralPower);
        double bl = (axialPower - lateralPower);
        double br = (axialPower + lateralPower);


        fl += turnPower;
        fr -= turnPower;
        bl += turnPower;
        br -= turnPower;

        left.setPower(parent.getFront() * fl);
        right.setPower(parent.getFront() * fr);
        left2.setPower(parent.getFront() * bl);
        right2.setPower(parent.getFront() * br);
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

        //parent.odometry.pause();

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
        //parent.odometry.play();
    }


    @Override
    public void run() {
        while(opModeIsActive() && !isInterrupted()) {
            //
        }
    }
}
