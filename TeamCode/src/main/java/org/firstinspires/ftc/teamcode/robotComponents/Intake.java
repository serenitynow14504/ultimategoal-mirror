package org.firstinspires.ftc.teamcode.robotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

@Config
public class Intake extends Capability {
    public ExpansionHubMotor intakeWheels;
    //private DistanceSensor stuckSensor;
    private final double CURRENT_THRESHOLD = 10;
    private ElapsedTime stuckTime, unStuckTime, startUp, retryTimer;
    private boolean on = false;
    private boolean doneIntaking = true, firstRingSeen = false, intaking = false;
    private int ringsIntaken = 0, retryCounter = 0;


    public Intake(Robot parent) {
        super(parent);
    }

    public void init(HardwareMap hardwareMap) {
        intakeWheels = (ExpansionHubMotor)hardwareMap.get("intake");
        stuckTime = new ElapsedTime();
        unStuckTime = new ElapsedTime();
        startUp = new ElapsedTime();
        retryTimer = new ElapsedTime();
    }

    public void teleOp(Gamepad gamepad1, Gamepad gamepad2) {
        //intakeWheels.setPower(gameplay.right_stick_y);
        if(gamepad2.left_stick_y>0.5) {
            on();
            robot.setLedColors(0,255,0);
        } else if(gamepad2.left_stick_y<-0.5 || gamepad2.left_trigger>=0.9) {
            off();
        } else if(gamepad2.left_bumper) {
            reverse();
        }
    }

    public void on() {
        intakeWheels.setPower(1);
        on = true;
    }

    public void reverse() {
        intakeWheels.setPower(-1);
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
//        stuckTime.reset();
//        unStuckTime.reset();

        /*while(opModeIsActive()) {
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

            if (intakeWheels.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > CURRENT_THRESHOLD) {
                intakeWheels.setPower(0);
                parent.setLedColors(255, 0, 0);
            }
        }*/
    }

    @Override
    void tick(TelemetryPacket packet) {
        packet.put("intake current draw: ", intakeWheels.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
    }


}
