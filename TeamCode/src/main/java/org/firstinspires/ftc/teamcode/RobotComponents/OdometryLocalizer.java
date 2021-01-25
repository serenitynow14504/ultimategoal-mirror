package org.firstinspires.ftc.teamcode.RobotComponents;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.VectorD;


public class OdometryLocalizer extends Capability{
    private DcMotor encX, encY;
    private boolean active = true;


    public enum MODES {
        ARC, LINE, BAD
    }

    private MODES mode = MODES.LINE;


    private int oldX, oldY, newX, newY;
    private double oldR, newR;

    private ElapsedTime speedTimer;
    private double updateSpeed = -1;


    private double ePosX, ePosY, eMag, eAng;    //xp, yp, le, theta e

    private double xSpeed = 0, ySpeed = 0;



    public OdometryLocalizer(Robot par, double xp, double yp) {
        super(par);
        speedTimer = new ElapsedTime();

        this.ePosX = xp;  //xp
        this.ePosY = yp;  //yp
        eMag = Math.sqrt(Math.pow(ePosX, 2) + Math.pow(ePosY, 2));   //le
        eAng = Math.atan2(ePosY, ePosX);   //theta e

    }

    public void setMode(MODES m) {mode = m;}
    public MODES getMode() {return mode;}

    void pause() {
        active = false;
    }

    void play() {
        active = true;
    }

    void toggle() {
        active = !active;
    }

    double getEncPos(int dim) {
        if(dim == 0) {return encX.getCurrentPosition();}
        else {return encY.getCurrentPosition();}
    }

    @Override
    public void run() {

        //Utilities.log("Odometry started");

        oldY = getOdoEncoderYPosition();

        while(opModeIsActive()) {
            speedTimer.reset();
            newX = getOdoEncoderXPosition();
            newY = getOdoEncoderYPosition();
            newR = Math.toRadians(parent.imu.getAngle());

            if(active) {
                switch (mode) {
                    case ARC:
                        arcOdo();
                        break;
                    case LINE:
                        lineOdo();
                        break;
                    case BAD:
                        badOdo();
                        break;
                }
            }

            xSpeed = encoderCountsToInches(newX - oldX) / speedTimer.seconds();
            ySpeed = encoderCountsToInches(newY - oldY) / speedTimer.seconds();

            //parent.setPosition(new VectorD(parent.getPosition().get(0),
            //        parent.getPosition().get(1), (float)parent.imu.getAngle()));

            oldX = newX;
            oldY = newY;
            oldR = newR;


            updateSpeed = speedTimer.milliseconds();
            speedTimer.reset();
        }
    }

    double getxSpeed() {
        return xSpeed;
    }

    double getySpeed() {
        return ySpeed;
    }

    void arcOdo() {
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
    }

    void lineOdo() {
        //parent.imu.setAngles();

        int xl = newX - oldX;
        int yl = newY - oldY;

        double xInches = encoderCountsToInches(xl);     //same
        double yInches = encoderCountsToInches(yl);     //same

        double finalX = -(yInches*Math.sin(newR) + xInches*Math.cos(newR));
        double finalY = yInches*Math.cos(newR) - xInches*Math.sin(newR);

        //double finalX = xInches*Math.cos(newR) - yInches*Math.sin(newR);
        //double finalY = yInches*Math.cos(newR) + xInches*Math.sin(newR);

        VectorD currentPos = parent.getPose();
        VectorD newPos = new VectorD((float)(currentPos.get(0) + finalX),
                (float)(currentPos.get(1) + finalY), (float)Math.toDegrees(newR));

        parent.setPosition(newPos);
        //log("odometry: " + newPos);
    }

    void badOdo() {
        double actualMoveX = encoderCountsToInches(newX - oldX);
        double actualMoveY = encoderCountsToInches(newY - oldY);

        double t = Math.toRadians(parent.getTargetRotation());
        double globalMoveX = actualMoveX*Math.cos(t) - actualMoveY*Math.sin(t);
        double globalMovey = actualMoveX*Math.sin(t) + actualMoveY*Math.cos(t);


        VectorD currentPosition = parent.getPose();

        VectorD globalMove = new VectorD((float)globalMoveX, (float)globalMovey);

        currentPosition.add(globalMove);

        parent.setPosition(currentPosition);
    }

    double getUpdateSpeed() {
        return updateSpeed;
    }

    public int getOdoEncoderXPosition() {return encX.getCurrentPosition();}
    public int getOdoEncoderYPosition() {return encY.getCurrentPosition();}

    void init(HardwareMap hardwareMap) {
        encX = hardwareMap.get(DcMotor.class, "encX");
        encY = hardwareMap.get(DcMotor.class, "encY");
        oldX = getOdoEncoderXPosition();
        oldY = getOdoEncoderYPosition();
        parent.imu.setAngles();
        oldR = Math.toRadians(parent.imu.heading());
    }

    void teleOp(Gamepad gamepad1, Gamepad gamepad2) {}
}
