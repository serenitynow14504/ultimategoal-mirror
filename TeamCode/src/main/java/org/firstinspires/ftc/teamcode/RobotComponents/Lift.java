package org.firstinspires.ftc.teamcode.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.Line;
import org.firstinspires.ftc.teamcode.Common.PIDController;
import org.firstinspires.ftc.teamcode.Common.Util;
import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.openftc.revextensions2.ExpansionHubMotor;

@Config
public class Lift extends Capability {
    private HardwareMap hardwareMap;
    private ExpansionHubMotor platform;
    private Rev2mDistanceSensor platformSensor;
    private final int POSITION_DIFFERENCE = 6750, liftPosThresh = 60;//8924
    private int bottomPos, topPos;
    private double targetLiftPos;
    public boolean usingEncoder = false;

    private boolean sensorDied = false;

    private final double PULLEY_DIAMETER = 30;//mm
    private final double CLICKS_PER_REV = 537.6;//*2;//8192 for odo encoder
    private final double MM_PER_CLICK = (Math.PI * PULLEY_DIAMETER) / CLICKS_PER_REV;

    public static double LiftKP = 0.008;//0.00675
    public static double LiftKI = 0;
    public static double LiftKD = 0.002;//0.0015

    public static double LiftTopPosDist = 107;//118
    public static double LiftBottomPosDist = 50;
    private boolean liftUp = true;

    private double initialDist;
    private Line encoderToDist;

    private PIDController liftPID;
    private double PIDPow = 0;


    public Lift(Robot parent) {
        super(parent);
    }

    void init(HardwareMap hardwareMap) {
        platform = (ExpansionHubMotor)hardwareMap.get("platform");
        platform.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        platform.setDirection(DcMotorSimple.Direction.REVERSE);
        platformSensor = (Rev2mDistanceSensor)hardwareMap.get(DistanceSensor.class, "liftHeight");

        this.hardwareMap = hardwareMap;
        topPos = liftEncoderPos();
        bottomPos = topPos - POSITION_DIFFERENCE;

        targetLiftPos = LiftTopPosDist;

        int ct = 0;
        double avg = 0;
        do {
            initialDist = platformSensor.getDistance(DistanceUnit.MM);
            Util.log("sensor: " + initialDist);
            if(validDist(initialDist)) {
                avg += initialDist;
                ct++;
            }
        } while(ct < 10);
        avg/=ct;
        encoderToDist = new Line(new VectorD(liftEncoderPos(), avg), MM_PER_CLICK);

        liftPID = new PIDController(LiftKP, LiftKI, LiftKD);
        liftPID.setOutputRange(0, 5);
        liftPID.setSetpoint(topPos);
        liftPID.enable();
    }

    public void teleOp(Gamepad gamepad1, Gamepad gamepad2) {
        if(gamepad2.left_stick_y<-0.5) {
            liftUp();
        }
        if(gamepad2.left_stick_y>0.5) {
            liftDown();
        }
    }

    public void debugOp(Gamepad gamepad1, Gamepad gamepad2) {
        telemetry.addData("lift safe dist: ", getLiftHeightMM());
        telemetry.addData("lift dist: ", platformSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("lift encoder 'dist': ", encoderToDist.yFromX(liftEncoderPos()));
        telemetry.addData("lift pos: ", liftEncoderPos());
        telemetry.addData("lift target pos: ", targetLiftPos);
        telemetry.addData("lift power: ", PIDPow);
        telemetry.addData("g2 left stick y: ", gamepad2.left_stick_y);
        telemetry.update();

        liftPID.disable();
        PIDPow = 0;
        platform.setPower(gamepad2.left_stick_y/3);
    }

    public double getLiftHeightMM() {
        usingEncoder = false;
        double distance = platformSensor.getDistance(DistanceUnit.MM);
        if(validDist(distance)) return distance;
        usingEncoder = true;
        sensorDied = true;
        parent.setLedColors(255, 0, 0);
        return encoderToDist.yFromX(liftEncoderPos());
    }
    
    private int liftEncoderPos() {
        return -platform.getCurrentPosition();
    }

    private boolean validDist(double d) {
        return (d<LiftTopPosDist+40 && d>0);
    }


    public void liftUp() {
        targetLiftPos = LiftTopPosDist;
        liftUp = true;
    }

    public void liftDown() {
        targetLiftPos = LiftBottomPosDist;
        liftUp = false;
    }

    public boolean getLiftState() {
        return liftUp;
    }

    public boolean onTarget() {
        return liftPID.onTarget();
    }

    @Override
    public void run() {
        while(opModeIsActive()) {
            if(liftPID.isEnabled()) {
                liftPID.setSetpoint(targetLiftPos);
                PIDPow = -liftPID.performPID(getLiftHeightMM());
                //if(PIDPow < -0.3) PIDPow = -0.1;
                //if(PIDPow > 0.3) PIDPow = 0.1;
            }
            platform.setPower(PIDPow);

            /*if(sensorDied && Math.abs(PIDPow)<0.05) {
                platformSensor = (Rev2mDistanceSensor)hardwareMap.get(DistanceSensor.class, "liftHeight");
                sensorDied = false;
                parent.setLedColors(0, 255, 0);
            }*/
        }
    }
}
