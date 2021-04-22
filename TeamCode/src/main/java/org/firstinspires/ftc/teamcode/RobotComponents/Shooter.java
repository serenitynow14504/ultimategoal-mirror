package org.firstinspires.ftc.teamcode.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Common.Line;
import org.firstinspires.ftc.teamcode.Common.PolyLine;
import org.firstinspires.ftc.teamcode.Common.Util;
import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.openftc.revextensions2.ExpansionHubMotor;

@Config
public class Shooter extends Capability {
    private ExpansionHubMotor flyWheel;
    private PowerShotExecutor powerShotExecutor;
    private Servo pusherL, pusherR;//, flapL, flapR;
    private double power = 0;
    private boolean shooterOn = false;
    private final double CURRENT_THRESHOLD = 10;
    private boolean firePressed = false;
    private boolean fireRequested = false;
    private boolean fire3Pressed = false;
    private boolean fire3Requested = false;
    private boolean firePowPressed = false;
    private boolean firePowRequested = false;

    public static double P = 10, I = 3, D = 0, F = 0;

    private double targetVel = 0, setpointVel = 0;
    public static double onVelHigh = 345, onVelPower = 325;

    private boolean aimDown = false;

    private static final double RADIUS_INCHES = 4.5 / 2.54;



    public static double flapPos = 0.315, aimDownAmt = 0.0135;
    //"zero": 0.53

    //flap angle PLANE regression constants (z = ax + by + c)
    private static double flapUpRegA = -0.00000247433;
    private static double flapUpRegB = 0.0115096;//
    private static double flapUpRegC = -0.02;//0.18
    private static double flapUpRegD = -123.544;

    private static double LeftPusherRest = 0.15;
    private static double LeftPusherPush = 0.45;
    private static double RightPusherRest = 0.85;
    private static double RightPusherPush = 0.55;

    private Line LeftPushPos = new Line(new VectorD(0, LeftPusherRest),
            LeftPusherPush-LeftPusherRest);
    private Line RightPushPos = new Line(new VectorD(0, RightPusherRest),
            RightPusherPush-RightPusherRest);

    /*public static double flapDownRegA = -0.0000147484;
    public static double flapDownRegB = 0.0424102;
    public static double flapDownRegC = -0.202707;
    public static double flapDownRegD = -84.374;

    public static double powerV1 = 11.75;
    public static double powerS1 = 0.315;
    public static double powerV2 = 13.54;
    public static double powerS2 = 0.345;*/
    private PolyLine powerReg;



    public Shooter(Robot parent) {
        super(parent);
    }

    void init(HardwareMap hardwareMap) {
        flyWheel = (ExpansionHubMotor) hardwareMap.get("encY");
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        pusherL = hardwareMap.get(Servo.class, "pusherL");
        pusherR = hardwareMap.get(Servo.class, "pusherR");

        powerShotExecutor = new PowerShotExecutor();
        //flapL = hardwareMap.get(Servo.class, "flapL");
        //flapR = hardwareMap.get(Servo.class, "flapR");

        pushers(false);

        powerReg = new PolyLine(new VectorD[] {
                new VectorD(11.75, 0.315),
                new VectorD(13.54, 0.345),
                new VectorD(12.53, 0.335)
        });

        Util.log(flyWheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I
                , D, F));
        Util.log(flyWheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
    }

    public void setPower(double pow) {
        power = pow;

    }

    private void setVel(double v) {
        //flyWheel.setVelocity(v, AngleUnit.RADIANS);
        targetVel = v;
        //flyWheel.setMode(Run);
    }

    public void set(boolean state) {
        setVel(state ? (aimDown ? onVelPower : onVelHigh) : 0);
        shooterOn = state;
    }

    public void teleOp(Gamepad gamepad1, Gamepad gamepad2) {
        if(gamepad2.right_trigger>0.5 && !firePressed) {
            fireRequested = true;
            firePressed = true;
        }
        if(gamepad2.right_trigger<0.5) firePressed = false;

        if(gamepad2.right_bumper && !fire3Pressed) {
            fire3Requested = true;
            fire3Pressed = true;
        }
        if(!gamepad2.right_bumper) fire3Pressed = false;

        if(gamepad2.right_stick_x<-0.5 && !firePowPressed) {
            autoPowerShot();
            firePowPressed = true;
        }
        if(gamepad2.right_stick_x>-0.5) firePowPressed = false;


        //pushers(gamepad2.right_bumper);

        /*if(gamepad2.right_trigger>0.9) {
            holdPush();
        }*/


        if(gamepad2.x || gamepad2.y || gamepad2.b)  {
            aimPowerShot();
        } else if(gamepad2.a) {
            aimHighGoal();
        }


        telemetry.update();
        if(gamepad2.left_stick_y<-0.5) {
            set(true);
        }
        if(gamepad2.left_stick_y>0.5 || gamepad2.left_trigger>=0.9) {
            set(false);
        }

        if(gamepad2.right_stick_button) {
            try {
                powerShotExecutor.interrupt();
            } catch(Exception e) {
                Util.log(e.toString());
            }
        }
    }

    public void push() {
        if(!parent.lift.getLiftState()) return;
        pushers(true);
        sleep(300);
        pushers(false);
        sleep(750);
    }

    public void autoPowerShot() {
        firePowRequested = true;
    }

    
    public void pushers(double pos) {
//        pusherL.setPosition(pos);
//        pusherR.setPosition(1-pos);
        pusherL.setPosition(LeftPushPos.yFromX(pos));
        pusherR.setPosition(RightPushPos.yFromX(pos));
    }

    public void pushers(boolean state) {
        pushers(state ? 1 : 0);
    }

    /*public void flap(double pos) {
        flapL.setPosition(1-pos);
        flapR.setPosition(pos);
    }

    private double calcFlapPos(boolean powerShot) {
        double dist = parent.getAimTargetDist();
        double volt = parent.getBatteryVoltage();

        *//*if(powerShot) {

            double a = flapUpRegA;
            double b = flapUpRegB;
            double c = flapUpRegC;
            double d = flapUpRegD;

            return a * (dist + d) * (dist + d) + b * volt + c;
            //return dist*flapRegA + volt*flapRegB + flapRegC + (powerShot ? aimDownAmt : 0);
        } else {
            return powerReg.yFromX(volt);
        }*//*

        if(powerShot) {
            return 0.59 + 0.042/(1d+Math.pow(0.23, volt-12.8));
        } else {
            return 0.548 + 0.042/(1d+Math.pow(0.2, volt-12.7));
        }
    }*/

    public void aimPowerShot() {
        aimDown = true;
        set(shooterOn);
    }

    public void aimHighGoal() {
        aimDown = false;
        set(shooterOn);
    }

    /**@return speed of the flywheel in radians per second**/
    public double getVel() {
        return flyWheel.getVelocity(AngleUnit.RADIANS);
    }

    @Override
    public void run() {
        while(opModeIsActive()) {
            /*double newPow = flyWheel.getPower();
            if(newPow < power - 0.05) {
                newPow += 0.075;
            } else if(newPow > power + 0.05) {
                newPow -= 0.075;
            } else {
                newPow = power;
            }

            flyWheel.setPower(newPow);*/

            if(setpointVel < targetVel - 26) {
                setpointVel += 50;
            } else if(setpointVel > targetVel + 26) {
                setpointVel -= 50;
            } else {
                setpointVel = targetVel;
            }

            flyWheel.setVelocity(setpointVel, AngleUnit.RADIANS);

            /*if (flyWheel.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > CURRENT_THRESHOLD) {
                setPower(0);
                parent.setLedColors(255, 0, 0);
            }*/

            if(fireRequested) {
                push();
                fireRequested = false;
            }
            if(fire3Requested) {
                push();
                push();
                push();
                fire3Requested = false;
            }
            if(firePowRequested) {
                powerShotExecutor.start();
                firePowRequested = false;
            }

            ///flap(calcFlapPos(aimDown));
            //flap(0.53);
        }
    }

    private class PowerShotExecutor extends Thread {
        @Override
        public void run() {
            aimPowerShot();
            //flap(calcFlapPos(aimDown));
            parent.aim();
            parent.setAimPos(FieldConstants.LEFT_POWER_SHOT);
            while(!parent.driveTrain.heading.onTarget()) {
                Shooter.this.sleep(50);
            }
            Shooter.this.sleep(250);
            push();
            parent.setAimPos(FieldConstants.MID_POWER_SHOT);
            while(!parent.driveTrain.heading.onTarget()) {
                Shooter.this.sleep(50);
            }
            Shooter.this.sleep(250);
            push();
            parent.setAimPos(FieldConstants.RIGHT_POWER_SHOT);
            while(!parent.driveTrain.heading.onTarget()) {
                Shooter.this.sleep(50);
            }
            Shooter.this.sleep(250);
            push();
            parent.stopAim();

            aimHighGoal();
        }
    }

}
