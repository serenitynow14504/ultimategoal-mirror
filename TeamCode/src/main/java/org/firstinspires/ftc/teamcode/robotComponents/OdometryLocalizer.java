package org.firstinspires.ftc.teamcode.robotComponents;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.common.Util;
import org.firstinspires.ftc.teamcode.common.VectorD;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;


public abstract class OdometryLocalizer extends Capability {
    private boolean active = true;
    protected boolean firstTime = true;



    private ElapsedTime speedTimer;
    private double updateSpeed = -1;

    double encoderCountsToInches(int counts) {
        return RobotConstants.COUNTS_TO_INCHES_CONVERSION * counts;
        //return ((double)counts/Robot.ODO_ENCODER_COUNTS_PER_REV) * Robot.ODO_WHEEL_CIRC;
    }

    //private final double ePosX, ePosY, eMag, eAng;    //xp, yp, le, theta e

    private double xSpeed = 0, ySpeed = 0;



    public OdometryLocalizer(Robot par) {
        super(par);
        speedTimer = new ElapsedTime();

        /*this.ePosX = xp;  //xp
        this.ePosY = yp;  //yp
        eMag = Math.sqrt(Math.pow(ePosX, 2) + Math.pow(ePosY, 2));   //le
        eAng = Math.atan2(ePosY, ePosX);   //theta e*/

    }


    void pause() {
        active = false;
        firstTime = true;
    }


    void play() {
        active = true;
    }

    void toggle() {
        active = !active;
    }


    @Override
    public void run() {
    }

    @Override
    public void tick(TelemetryPacket packet) {
        speedTimer.reset();
        if(active) {
            VectorD tryGlobal = calculateGlobal(robot.getPose(), packet);
            if (tryGlobal != null) {
                robot.setPose(tryGlobal);
            } else {
                RealVector local = calculate();
                VectorD oldPose = robot.getPose();
                double r = Math.toRadians(oldPose.getZ()) + local.getEntry(2);//now uses NEW rot
                RealMatrix rotation = Util.getPoseRotationMatrix(r);
                RealVector global = rotation.operate(local);
                //Util.log("odo adds: "+global.toString());
                robot.setPose(oldPose.added(new VectorD(global.getEntry(0),
                        global.getEntry(1), Math.toDegrees(global.getEntry(2)))));
            }
        }
        updateSpeed = speedTimer.milliseconds();
    }

    abstract void setHeading(double degrees);

    abstract protected RealVector calculate();//returns heading in radians

    protected VectorD calculateGlobal(VectorD oldPose, TelemetryPacket packet) {
        return null;
    }

    double getUpdateSpeed() {
        return updateSpeed;
    }

    abstract void init(HardwareMap hardwareMap);

    void teleOp(Gamepad gamepad1, Gamepad gamepad2) {}
}
