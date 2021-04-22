package org.firstinspires.ftc.teamcode.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

@Config
public class Intake extends Capability {
    public ExpansionHubMotor intakeWheels;
    private Servo tilt;
    //private DistanceSensor stuckSensor;
    private final double CURRENT_THRESHOLD = 10;
    private ElapsedTime stuckTime, unStuckTime, startUp, retryTimer;
    private boolean on = false;
    private boolean doneIntaking = true, firstRingSeen = false, intaking = false;
    private int ringsIntaken = 0, retryCounter = 0;

    public static double tiltOn = 0.4, tiltOff = 1;

    public Intake(Robot parent) {
        super(parent);
    }

    public void init(HardwareMap hardwareMap) {
        intakeWheels = (ExpansionHubMotor)hardwareMap.get("encX");
        tilt = hardwareMap.get(Servo.class, "mask");
        //stuckSensor = hardwareMap.get(DistanceSensor.class, "ramp");
        stuckTime = new ElapsedTime();
        unStuckTime = new ElapsedTime();
        startUp = new ElapsedTime();
        retryTimer = new ElapsedTime();
        tilt(false);
    }

    public void teleOp(Gamepad gamepad1, Gamepad gamepad2) {
        //intakeWheels.setPower(gamepad.right_stick_y);
        if(gamepad2.left_stick_y>0.5) {
            on();
            parent.setLedColors(0,255,0);
        } else if(gamepad2.left_stick_y<-0.5 || gamepad2.left_trigger>=0.9) {
            off();
        } else if(gamepad2.left_bumper) {
            reverse();
        }

        if(gamepad1.x) {
            tilt(false);
        } else if(gamepad1.y) {
            tilt(true);
        }
    }

    public void tilt(boolean state) {
        tilt.setPosition(state ? tiltOn : tiltOff);
    }

    public void on() {
        intakeWheels.setPower(0.5);
        on = true;
        doneIntaking = false;
        firstRingSeen = false;
        ringsIntaken = 0;
        retryCounter = 0;
        startUp.reset();
        retryTimer.reset();
    }

    public void full() {
        on = true;
        intakeWheels.setPower(1);
    }

    public int ringsIntaken() {
        return ringsIntaken;
    }

    public void reverse() {
        intakeWheels.setPower(-0.5);
    }

    public void off() {
        on = false;
        intakeWheels.setPower(0);
    }


    public boolean doneIntaking() {
        return !intakingRing() || isStopRequested() || !opModeIsActive();
    }

    public boolean intakingRing() {
        return intaking;
    }

    private double preCurr = 0, prePeakCurr = 0;
    private boolean peakRising = false;

    @Override
    public void run() {
        tilt(false);
        stuckTime.reset();
        unStuckTime.reset();
        while(opModeIsActive()) {
            /*if(startUp.milliseconds() > 500 && intakeWheels.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > 3) {
                firstRingSeen = true;
                retryTimer.reset();
            }*/

            double currcurr = intakeWheels.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
            //current current
            if(currcurr < preCurr) {
                peakRising = false;
                prePeakCurr = preCurr;
            }
            if(currcurr > preCurr) {
                peakRising = true;
            }
            preCurr = currcurr;

            intaking = prePeakCurr>1.05;

            /*if(ringStuck()) {
                firstRingSeen = true;
                doneIntaking = false;
                unStuckTime.reset();
                if(stuckTime.milliseconds()>500) {
                    if(parent.lift.getLiftState() || retryCounter > 4) {
                        ejectRing();
                    } else {
                        reTakeRing();
                    }
                    if(retryTimer.milliseconds() < 1000) retryCounter++;
                    retryTimer.reset();
                }
            } else {
                stuckTime.reset();
                if(unStuckTime.milliseconds()>1200 && firstRingSeen) {
                    doneIntaking = true;
                }
            }
            if(retryTimer.milliseconds()>2000) {
                retryCounter = 0;
            }*/
            if (intakeWheels.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > CURRENT_THRESHOLD) {
                intakeWheels.setPower(0);
                parent.setLedColors(255, 0, 0);
            }
        }
    }
}
