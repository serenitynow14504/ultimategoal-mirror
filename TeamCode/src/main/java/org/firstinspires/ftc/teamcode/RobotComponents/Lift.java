package org.firstinspires.ftc.teamcode.RobotComponents;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.Line;
import org.firstinspires.ftc.teamcode.Common.PIDController;
import org.firstinspires.ftc.teamcode.Common.Utilities;
import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.openftc.revextensions2.ExpansionHubMotor;

public class Lift extends Capability {
    private HardwareMap hardwareMap;
    private ExpansionHubMotor platform;
    private Rev2mDistanceSensor platformSensor;
    private final int POSITION_DIFFERENCE = 6750, liftPosThresh = 60;//8924
    private int bottomPos, topPos;
    private double targetLiftPos;

    private boolean sensorDied = false;

    private final double PULLEY_DIAMETER = 30;//mm
    private final int CLICKS_PER_REV = 8192;//2048
    private final double MM_PER_CLICK = (Math.PI * PULLEY_DIAMETER) / CLICKS_PER_REV;


    public static double LiftKP = 0.00675;
    public static double LiftKI = 0;
    public static double LiftKD = 0.0015;

    public static double LiftTopPosDist = 143;
    public static double LiftBottomPosDist = 50;

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
        topPos = platform.getCurrentPosition();
        bottomPos = topPos - POSITION_DIFFERENCE;

        targetLiftPos = LiftTopPosDist;

        do {
            initialDist = platformSensor.getDistance(DistanceUnit.MM);
            Utilities.log("sensor: " + initialDist);
        } while(!validDist(initialDist));
        encoderToDist = new Line(new VectorD(platform.getCurrentPosition(), initialDist), MM_PER_CLICK);

        liftPID = new PIDController(LiftKP, LiftKI, LiftKD);
        liftPID.setOutputRange(0, 5);
        liftPID.setSetpoint(topPos);
        liftPID.enable();
    }

    public void teleOp(Gamepad gamepad1, Gamepad gamepad2) {
        /*telemetry.addData("lift safe dist: ", getLiftHeightMM());
        telemetry.addData("lift dist: ", platformSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("lift encoder 'dist': ", encoderToDist.yFromX(platform.getCurrentPosition()));
        telemetry.addData("lift pos: ", platform.getCurrentPosition());
        telemetry.addData("lift target pos: ", targetLiftPos);
        telemetry.addData("lift power: ", PIDPow);
        telemetry.addData("g2 left stick y: ", gamepad2.left_stick_y);
        telemetry.update();*/

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
        telemetry.addData("lift encoder 'dist': ", encoderToDist.yFromX(platform.getCurrentPosition()));
        telemetry.addData("lift pos: ", platform.getCurrentPosition());
        telemetry.addData("lift target pos: ", targetLiftPos);
        telemetry.addData("lift power: ", PIDPow);
        telemetry.addData("g2 left stick y: ", gamepad2.left_stick_y);
        telemetry.update();

        liftPID.disable();
        PIDPow = 0;
        platform.setPower(gamepad2.left_stick_y/3);
    }

    private double getLiftHeightMM() {
        double distance = platformSensor.getDistance(DistanceUnit.MM);
        if(validDist(distance)) return distance;
        sensorDied = true;
        parent.setLedColors(255, 0, 0);
        return encoderToDist.yFromX(platform.getCurrentPosition());
    }

    private boolean validDist(double d) {
        return (d<LiftTopPosDist+20 && d>0);
    }


    public void liftUp() {
        targetLiftPos = LiftTopPosDist;
    }

    public void liftDown() {
        targetLiftPos = LiftBottomPosDist;
    }

    @Override
    public void run() {
        while(opModeIsActive()) {
            if(liftPID.isEnabled()) {
                liftPID.setSetpoint(targetLiftPos);
                PIDPow = liftPID.performPID(getLiftHeightMM());
            }
            platform.setPower(PIDPow);

            if(sensorDied && Math.abs(PIDPow)<0.05) {
                platformSensor = (Rev2mDistanceSensor)hardwareMap.get(DistanceSensor.class, "liftHeight");
                sensorDied = false;
                parent.setLedColors(0, 255, 0);
            }
        }
    }
}
