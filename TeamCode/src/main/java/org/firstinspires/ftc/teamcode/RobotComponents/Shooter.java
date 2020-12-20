package org.firstinspires.ftc.teamcode.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.PIDController;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

@Config
public class Shooter extends Capability {
    private ExpansionHubMotor flyWheel, platform;
    private Servo pusherL, pusherR, flapL, flapR;
    private Rev2mDistanceSensor platformSensor;
    private double power = 0;
    private final double CURRENT_THRESHOLD = 10;
    private final int POSITION_DIFFERENCE = 6750, liftPosThresh = 60;//8924
    private int bottomPos, topPos;
    private double targetLiftPos;
    private boolean firePressed = false, flyWheelPressed = false;
    private boolean pushRequested = false;
    private boolean aimDown = false;

    //out = kP*e + de/dt*kD

    public static double flapDefault = 0.3, flapPos = 0.315, aimDownAmt = 0.0135;
    //flap angle PLANE regression constants (z = ax + by + c)
    public static final double flapRegA = 0.0000828387;
    public static final double flapRegB = 0.0197579;
    public static final double flapRegC = 0.0579327;

    public static double LiftKP = 0.0075;
    public static double LiftKI = 0;
    public static double LiftKD = 0.0015;

    public static double LiftTopPosDist = 143;
    public static double LiftBottomPosDist = 50;

    private PIDController liftPID;
    private double PIDPow = 0;


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
        flapL = hardwareMap.get(Servo.class, "flapL");
        flapR = hardwareMap.get(Servo.class, "flapR");
        platformSensor = (Rev2mDistanceSensor)hardwareMap.get(DistanceSensor.class, "liftHeight");

        topPos = platform.getCurrentPosition();
        bottomPos = topPos - POSITION_DIFFERENCE;

        targetLiftPos = LiftTopPosDist;
        pusherL.setPosition(0.9);
        pusherR.setPosition(0);

        liftPID = new PIDController(LiftKP, LiftKI, LiftKD);
        liftPID.setOutputRange(0, 5);
        liftPID.setSetpoint(topPos);
        liftPID.enable();

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

        /*if(gamepad.right_stick_button && !flyWheelPressed) {
            if(power < -0.5) {
                setPower(0);
            } else {
                setPower(-1);
            }
            flyWheelPressed = true;
        }
        if(!gamepad.right_stick_button) {
            flyWheelPressed = false;
        }*/

        if(gamepad1.y) {
            aimPowerShot();
        } else {
            aimHighGoal();
        }


        telemetry.addData("lift dist: ", platformSensor.getDistance(DistanceUnit.MM));
        telemetry.addLine();
        telemetry.addData("lift pos: ", platform.getCurrentPosition());
        telemetry.addData("lift target pos: ", targetLiftPos);
        telemetry.addData("lift power: ", PIDPow);

        telemetry.update();
        if(gamepad2.left_stick_y<-0.5) {
            liftUp();
        }
        if(gamepad2.left_stick_y>0.5) {
            liftDown();
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

    void liftUp() {
        targetLiftPos = LiftTopPosDist;
        setPower(-1);
    }

    void liftDown() {
        targetLiftPos = LiftBottomPosDist;
        setPower(0);
    }

    public void flap(double pos) {
        flapL.setPosition(1-pos);
        flapR.setPosition(pos);
    }

    private double calcFlapPos(boolean adjustForPowerShot) {
        double dist = adjustForPowerShot ? parent.getPowerShotDist() : parent.getGoalDist();
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

            liftPID.setSetpoint(targetLiftPos);
            PIDPow = liftPID.performPID(platformSensor.getDistance(DistanceUnit.MM));
            platform.setPower(PIDPow);
            /*if(platform.getCurrentPosition() < targetLiftPos - liftPosThresh) {
                platform.setPower(0.2);
            } else if (platform.getCurrentPosition() > targetLiftPos + liftPosThresh) {
                platform.setPower(-0.2);
            } else {
                platform.setPower(0);
            }*/

            if(pushRequested) {
                push();
                pushRequested = false;
            }

            flap(calcFlapPos(aimDown));
        }
    }
}
