package org.firstinspires.ftc.teamcode.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

@Config
public class Shooter extends Capability {
    private ExpansionHubMotor flyWheel;
    private Servo pusherL, pusherR, flapL, flapR;
    private double power = 0;
    private final double CURRENT_THRESHOLD = 10;
    private boolean firePressed = false, flyWheelPressed = false;
    private boolean pushRequested = false;
    private boolean aimDown = false;


    public static double flapDefault = 0.3, flapPos = 0.315, aimDownAmt = 0.0135;
    //flap angle PLANE regression constants (z = ax + by + c)
    public static final double flapRegA = 0.0000828387;
    public static final double flapRegB = 0.0197579;
    public static final double flapRegC = 0.0579327;


    public Shooter(Robot parent) {
        super(parent);
    }

    void init(HardwareMap hardwareMap) {
        flyWheel = (ExpansionHubMotor) hardwareMap.get("encY");
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pusherL = hardwareMap.get(Servo.class, "pusherL");
        pusherR = hardwareMap.get(Servo.class, "pusherR");
        flapL = hardwareMap.get(Servo.class, "flapL");
        flapR = hardwareMap.get(Servo.class, "flapR");

        pusherL.setPosition(0.9);
        pusherR.setPosition(0);

        flap(flapDefault);
    }

    public void setPower(double pow) {
        power = pow;
    }

    public void teleOp(Gamepad gamepad1, Gamepad gamepad2) {
        if(gamepad1.right_bumper && !firePressed) {
            pushRequested = true;
            firePressed = true;
        }
        if(!gamepad1.right_bumper) firePressed = false;

        if(gamepad1.y) {
            aimPowerShot();
        } else {
            aimHighGoal();
        }


        telemetry.update();
        if(gamepad2.left_stick_y<-0.5) {
            setPower(-1);
        }
        if(gamepad2.left_stick_y>0.5) {
            setPower(0);
        }
    }

    public void push() {
        pusherL.setPosition(0.8);
        sleep(100);
        pusherR.setPosition(0.5);
        sleep(300);
        pusherR.setPosition(0);
        sleep(100);
        pusherL.setPosition(0.9);
    }

    public void flap(double pos) {
        flapL.setPosition(1-pos);
        flapR.setPosition(pos);
    }

    private double calcFlapPos(boolean adjustForPowerShot) {
        double dist = parent.getAimTargetDist();
        double volt = parent.getBatteryVoltage();
        return dist*flapRegA + volt*flapRegB + flapRegC + (adjustForPowerShot ? aimDownAmt : 0);
    }

    public void aimPowerShot() {aimDown = true;}

    public void aimHighGoal() {aimDown = false;}

    @Override
    public void run() {
        while(opModeIsActive()) {
            double newPow = flyWheel.getPower();
            if(newPow < power) {
                newPow += 0.075;
            } else if(newPow > power) {
                newPow -= 0.075;
            }

            flyWheel.setPower(newPow);
            if (flyWheel.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > CURRENT_THRESHOLD) {
                setPower(0);
                parent.setLedColors(255, 0, 0);
            }

            if(pushRequested) {
                push();
                pushRequested = false;
            }

            flap(calcFlapPos(aimDown));
        }
    }
}
