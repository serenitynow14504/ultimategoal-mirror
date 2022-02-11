package org.firstinspires.ftc.teamcode.robotComponents;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.common.Util;
import org.firstinspires.ftc.teamcode.common.VectorD;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;

import java.util.Locale;


public class TwoWheelOdometryOld extends OdometryLocalizer{
    private DcMotor encX, encY;
    private Imu imu;
    private boolean active = true;
    private final Object lock = new Object();

    private int oldX, oldY;
    private double oldR;



    private double ePosX, ePosY, eMag, eAng;    //xp, yp, le, theta e


    public TwoWheelOdometryOld(Robot par) {
        super(par);
        ePosX = RobotConstants.FRONT_RIGHT_ODO_POSE.getX();  //xp
        ePosY = RobotConstants.SIDE_ODO_POSE.getY();  //yp
        eMag = Math.sqrt(ePosX*ePosX + ePosY*ePosY);   //le
        eAng = Math.atan2(ePosY, ePosX);   //theta e

    }

    double getEncPos(int dim) {
        if(dim == 0) {return encX.getCurrentPosition();}
        else {return encY.getCurrentPosition();}
    }

    /*void arcOdo() {
        //parent.imu.setAngles();

        int xl = newX - oldX;
        int yl = newY - oldY;

        double xInches = encoderCountsToInches(xl);     //same
        double yInches = encoderCountsToInches(yl);     //same

        double moveDir = Math.atan2(yInches, xInches);   //theta b
        double moveDist = Math.sqrt(xInches*xInches + yInches*yInches);   //l

        double thetaOld = oldR + moveDir;   //same
        double thetaNew = newR + moveDir;   //same

        double thetaMovementOld = oldR + eAng;
        double thetaMovementNew = newR + eAng;


        double xEncodersOld = Math.cos(thetaMovementOld) * eMag;   //xei
        double yEncodersOld = Math.sin(thetaMovementOld) * eMag;   //yei

        double xEncodersNew = Math.cos(thetaMovementNew) * eMag;   //xef
        double yEncodersNew = Math.sin(thetaMovementNew) * eMag;   //yef

        double thetaArc = thetaNew - thetaOld;   //theta d

        double finalX, finalY;   //xf, xy
        if(thetaArc != 0) {  //thetaArc != 0
            double radius = moveDist / thetaArc;   //r

            double vertX = radius * Math.cos(Math.PI/2*(thetaArc/Math.abs(thetaArc)) + thetaOld) + xEncodersOld;  //xv
            double vertY = radius * Math.sin(Math.PI/2*(thetaArc/Math.abs(thetaArc)) + thetaOld) + yEncodersOld;  //yv

            double thetaFinal = thetaOld + thetaArc + Math.PI + (Math.PI/2)*(thetaArc/Math.abs(thetaArc));  //theta f

            finalX = radius * Math.cos(thetaFinal) + vertX - xEncodersNew;
            finalY = radius * Math.sin(thetaFinal) + vertY - yEncodersNew;
        } else {
            finalX = xEncodersOld + moveDist * Math.cos(thetaOld);
            finalY = yEncodersOld + moveDist * Math.sin(thetaOld);
        }

        VectorD currentPos = parent.getPose();
        VectorD newPos = new VectorD((float)(currentPos.get(0) + finalX),
                (float)(currentPos.get(1) + finalY), (float)Math.toDegrees(newR));
        parent.setPosition(newPos);
    }*/


    protected RealVector calculate() { //line segment method
        //parent.imu.setAngles();

        int newX = getOdoEncoderXPosition();
        int newY = getOdoEncoderYPosition();
        double newRDegrees = imu.getAngle();
        double newRRadians = Math.toRadians(newRDegrees);

        int dx = newX - oldX;
        int dy = newY - oldY;
        double dr = newRRadians - oldR;

        double xInches = encoderCountsToInches(dx);     //same
        double yInches = encoderCountsToInches(dy);     //same

        double finalX = -(yInches*Math.sin(newRRadians) + xInches*Math.cos(newRRadians));
        double finalY = yInches*Math.cos(newRRadians) - xInches*Math.sin(newRRadians);

        //double finalX = xInches*Math.cos(newR) - yInches*Math.sin(newR);
        //double finalY = yInches*Math.cos(newR) + xInches*Math.sin(newR);

        //VectorD currentPos = parent.getPose();
//        VectorD newPos = new VectorD((float)(currentPos.get(0) + finalX),
//                (float)(currentPos.get(1) + finalY), (float)Math.toDegrees(newR));

        //parent.setPosition(newPos);
        //log("odometry: " + newPos);
        oldX = newX;
        oldY = newY;
        oldR = newRRadians;

        //return new VectorD(finalY, finalY, dr);
        return new ArrayRealVector(new double[]{xInches, yInches, dr});
    }

    protected VectorD calculateGlobal(VectorD oldPose, TelemetryPacket packet) {
        synchronized(lock) {
            packet.addLine("old pose: " + oldPose.toString());
            int newX = getOdoEncoderXPosition();
            int newY = getOdoEncoderYPosition();
            double newRDegrees = imu.getAngle();
            double newRRadians = Math.toRadians(newRDegrees);

            if (firstTime) {
                Util.log("first time");
                oldR = newRRadians;
                oldX = newX;
                oldY = newY;
                firstTime = false;
            }

            VectorD oldOffsetPos = Util.clipToXY(oldPose).added(new VectorD(eMag * Math.cos(eAng + oldR),
                    eMag * Math.sin(eAng + oldR)));
            packet.addLine("to: " + new VectorD(eMag * Math.cos(eAng + oldR),
                    eMag * Math.sin(eAng + oldR)));

            int dx = newX - oldX;
            int dy = newY - oldY;
            double dr = newRRadians - oldR;

            double xInches = encoderCountsToInches(dx);     //same
            double yInches = encoderCountsToInches(dy);     //same

            double finalX = -(yInches * Math.sin(newRRadians) + xInches * Math.cos(newRRadians));
            double finalY = yInches * Math.cos(newRRadians) - xInches * Math.sin(newRRadians);

            VectorD newOffsetPos = oldOffsetPos.added(new VectorD(finalX, finalY));
            VectorD newPos = newOffsetPos.subtracted(new VectorD(eMag * Math.cos(eAng + newRRadians),
                    eMag * Math.sin(eAng + newRRadians)));


            packet.addLine(String.format(Locale.getDefault(), "odo add: (%f, %f)", finalX, finalY));
            packet.addLine("back: " + new VectorD(eMag * Math.cos(eAng + newRRadians), eMag * Math.sin(eAng + newRRadians)).toString());
            packet.addLine("new pos: " + newPos.toString());
        /*double finalX = xInches*Math.cos(newR) - yInches*Math.sin(newR);
        double finalY = yInches*Math.cos(newR) + xInches*Math.sin(newR);

        VectorD currentPos = parent.getPose();
        VectorD newPos = new VectorD((float)(currentPos.get(0) + finalX),
                (float)(currentPos.get(1) + finalY), (float)Math.toDegrees(newR));

        parent.setPosition(newPos);
        log("odometry: " + newPos);*/

            oldX = newX;
            oldY = newY;
            oldR = newRRadians;

            return Util.addZ(newPos, newRDegrees);
        }
    }

    protected VectorD defaultEPos() {
        return new VectorD(eMag * Math.sin(eAng), eMag * Math.cos(eAng));
    }

    public void setHeading(double degrees) {
        imu.setHeading(degrees);
        //oldR = Math.toRadians(degrees);
    }

    public int getOdoEncoderXPosition() {return encX.getCurrentPosition();}
    public int getOdoEncoderYPosition() {return -encY.getCurrentPosition();}

    void init(HardwareMap hardwareMap) {
        encX = hardwareMap.get(DcMotor.class, RobotConstants.SIDE_ODO_NAME);//intake
        encY = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_RIGHT_ODO_NAME);//encY
        oldX = getOdoEncoderXPosition();
        oldY = getOdoEncoderYPosition();
        imu = new Imu(hardwareMap.get(BNO055IMU.class, "imu"), robot);
        imu.setAngles();
        oldR = Math.toRadians(imu.heading());
    }
}
