package org.firstinspires.ftc.teamcode.RobotComponents;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.PIDController;
import org.firstinspires.ftc.teamcode.Common.Utilities;
import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Path;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.RotationPath;

import androidx.annotation.NonNull;

@Deprecated
public class PurePursuitDrivetrain extends DriveTrain {
    private Path path;
    private RotationPath rotationPath;
    private Odometry.MODES mode;

    public PurePursuitDrivetrain(Robot p, Odometry.MODES mode) {
        super(p);
        this.mode = mode;
    }

    public void setPath(VectorD[] points) {
        path = new Path(points);
        if(parent.alliance == RobotConstants.ALLIANCES.BLUE) path.flipForBlue();
        if(points[1].length() == 3) {
            //log("creating rot path");
            rotationPath = new RotationPath(path, Utilities.extractZs(points));
        }
    }



    public void followPathNoPID(double power) {
        //if(parent.odometry.getMode() != Odometry.MODES.BAD) {return;}

        int origX = parent.odometry.getOdoEncoderXPosition();
        int origY = parent.odometry.getOdoEncoderYPosition();

        ElapsedTime timer = new ElapsedTime();


        PIDController r = new PIDController(RobotConstants.TRP, RobotConstants.TRI, RobotConstants.TRD);//p = 0.01   d = 0.006


        r.setSetpoint(parent.getTargetRotation());
        r.setOutputRange(0, power);  //-power to power?
        r.enable();


        double rad, ang, dist, rampDownDist = -1, rampDownTo = 0.35;
        double closestPathPointParameter;
        timer.reset();
        do {
            VectorD pos;
            if(mode == Odometry.MODES.LINE || mode == Odometry.MODES.ARC) pos = parent.getPosition();
            else pos = parent.getEstimatedPosition();

            path.calculateClosestData(pos);

            closestPathPointParameter = path.closestParameter();

            double lookAhead = RobotConstants.lookAheadConstant/(path.getClosestDist()/64.0 + 1.0);
            setProgress(closestPathPointParameter/path.getPathLength());
            lookAhead += closestPathPointParameter;

            VectorD pointFollow = path.pointFromParameter(lookAhead);


            double dx = pointFollow.get(0) - pos.get(0);
            double dy = pointFollow.get(1) - pos.get(1);


            double rot = Math.toRadians(parent.getTargetRotation()) + (Math.PI/2);

            ang = Math.atan2(dy, dx);

            rad = Math.sqrt(dy*dy + dx*dx);
            dist = Math.sqrt((path.getLastPoint().get(0) - pos.get(0)) * (path.getLastPoint().get(0) - pos.get(0)) + (path.getLastPoint().get(1) - pos.get(1)) * (path.getLastPoint().get(1) - pos.get(1)));


            if(lookAhead >= path.getPathLength() - 5 && rampDownDist == -1) {
                rampDownDist = rad;
            }

            double rampDownCoeff = 1;
            if(rampDownDist != -1) {
                rampDownCoeff = (1-rampDownTo)*rad/rampDownDist + rampDownTo;
            }

            double xPower = power*rampDownCoeff * Math.cos(ang + Math.PI/2 - rot);
            double yPower = power*rampDownCoeff * Math.sin(ang + Math.PI/2 - rot);



            double rCorrect = r.performPID(parent.imu.getAngle());


            setScaledPowersFromComponents(xPower, yPower, rCorrect);

            displayDash(parent.getPosition(), null, pointFollow);

            if(isInterrupted()) {
                resetInterrupt();
                break;
            }

            if(timer.milliseconds() > 10000) {
                parent.getMyOpMode().stop();
            }

        } while(opModeIsActive() && dist > 2 && closestPathPointParameter < path.getPathLength());

        setPowers(0);
        setProgress(0);
    }



    public void followPathAbsolutelyNoPID(double power, double rotPower) {
        //if(parent.odometry.getMode() != Odometry.MODES.BAD) {return;}

        ElapsedTime timer = new ElapsedTime();


        PIDController r = new PIDController(RobotConstants.TRP, RobotConstants.TRI, RobotConstants.TRD);//p = 0.01   d = 0.006


        r.setSetpoint(parent.getTargetRotation());
        r.setOutputRange(0, power);  //-power to power?
        r.enable();


        double rad, ang, rampDownDist = -1, rampDownTo = 0.35;
        double closestPathPointParameter;
        do {
            //timer.reset();

            VectorD pos;
            if(mode == Odometry.MODES.LINE || mode == Odometry.MODES.ARC) pos = parent.getPosition();
            else pos = parent.getEstimatedPosition();

            path.calculateClosestData(pos);

            closestPathPointParameter = path.closestParameter();

            double lookAhead = RobotConstants.lookAheadConstant/(path.getClosestDist()/64.0 + 1.0);
            setProgress(closestPathPointParameter/path.getPathLength());
            lookAhead += closestPathPointParameter;

            VectorD pointFollow = path.pointFromParameter(lookAhead);


            double dx = pointFollow.get(0) - pos.get(0);
            double dy = pointFollow.get(1) - pos.get(1);


            double rot = Math.toRadians(parent.imu.getAngle()) + (Math.PI/2);

            ang = Math.atan2(dy, dx);
            rad = Math.sqrt(dy*dy + dx*dx);


            if(lookAhead >= path.getPathLength() && rampDownDist == -1) {
                rampDownDist = rad;
            }

            double rampDownCoeff = 1;
            if(rampDownDist != -1) {
                rampDownCoeff = (1-rampDownTo)*rad/rampDownDist + rampDownTo;
            }

            double xPower = power*rampDownCoeff*Math.cos(ang + Math.PI/2 - rot);
            double yPower = power*rampDownCoeff*Math.sin(ang + Math.PI/2 - rot);



            double dRot = Math.atan2(rotationPath.rotFromParam(lookAhead) - parent.imu.getAngle(),
                    lookAhead - closestPathPointParameter);

            double rCorrect = Math.sin(dRot) * rotPower;



            /*double fl = yPower + xPower - rCorrect;
            double fr = yPower - xPower + rCorrect;
            double bl = yPower - xPower - rCorrect;
            double br = yPower + xPower + rCorrect;


            setPowers(fl, fr, bl, br);*/
            setScaledPowersFromComponents(xPower, yPower, rCorrect);
            displayDash(parent.getPosition(), null, pointFollow);

            if(isInterrupted()) {
                resetInterrupt();
                break;
            }

        } while(opModeIsActive() && rad > 2 && closestPathPointParameter != path.getPathLength());

        setPowers(0);
        setProgress(0);
    }





    void followPathALittlePID(double power) {
        //if(parent.odometry.getMode() != Odometry.MODES.BAD) {return;}

        int origX = parent.odometry.getOdoEncoderXPosition();
        int origY = parent.odometry.getOdoEncoderYPosition();

        ElapsedTime timer = new ElapsedTime();


        PIDController r = new PIDController(RobotConstants.TRP, RobotConstants.TRI, RobotConstants.TRD);//p = 0.01   d = 0.006


        r.setSetpoint(parent.getTargetRotation());
        r.setOutputRange(0, power);  //-power to power?
        r.enable();


        double rad, ang, rampDownDist = -1, rampDownTo = 0.35;
        double closestPathPointParameter;
        do {
            //timer.reset();

            VectorD pos;
            if(mode == Odometry.MODES.LINE || mode == Odometry.MODES.ARC) pos = parent.getPosition();
            else pos = parent.getEstimatedPosition();

            path.calculateClosestData(pos);

            closestPathPointParameter = path.closestParameter();

            double lookAhead = RobotConstants.lookAheadConstant/(path.getClosestDist()/64.0 + 1.0);
            setProgress(closestPathPointParameter/path.getPathLength());
            lookAhead += closestPathPointParameter;

            VectorD pointFollow = path.pointFromParameter(lookAhead);


            double dx = pointFollow.get(0) - pos.get(0);
            double dy = pointFollow.get(1) - pos.get(1);


            double rot = Math.toRadians(parent.getTargetRotation()) + (Math.PI/2);

            ang = Math.atan2(dy, dx);
            rad = Math.sqrt(dy*dy + dx*dx);


            if(lookAhead >= path.getPathLength() && rampDownDist == -1) {
                rampDownDist = rad;
            }

            double rampDownCoeff = 1;
            if(rampDownDist != -1) {
                rampDownCoeff = (1-rampDownTo)*rad/rampDownDist + rampDownTo;
            }

            double xPower = power*rampDownCoeff*Math.cos(ang + Math.PI/2 - rot);
            double yPower = power*rampDownCoeff*Math.sin(ang + Math.PI/2 - rot);



            double rCorrect = r.performPID(parent.imu.getAngle());

            /*double fl = yPower + xPower - rCorrect;
            double fr = yPower - xPower + rCorrect;
            double bl = yPower - xPower - rCorrect;
            double br = yPower + xPower + rCorrect;


            setPowers(fl, fr, bl, br);*/

            setScaledPowersFromComponents(xPower, yPower, rCorrect);

            displayDash(parent.getPosition(), null, pointFollow);

            if(isInterrupted()) {
                resetInterrupt();
                break;
            }

        } while(opModeIsActive() && rad > 2 && closestPathPointParameter != path.getPathLength());

        setPowers(0);
        setProgress(0);
    }




    public void displayDash(@NonNull VectorD position, VectorD estimatedPosition, VectorD lookAheadPoint) {
        if(parent.dashboard == null) return;
        TelemetryPacket packet = new TelemetryPacket();

        double cornerRad = Math.sqrt(parent.getWidth()*parent.getWidth()/4 + parent.getLength()*parent.getLength()/4);

        if(lookAheadPoint != null) {
            VectorD pos = position;
            if(estimatedPosition != null) pos = estimatedPosition;

            packet.fieldOverlay().setStrokeWidth(1).setStroke("black").strokePolyline(path.getXs(), path.getYs());
            packet.fieldOverlay().setStroke("green").strokeLine(pos.get(0), pos.get(1), lookAheadPoint.get(0), lookAheadPoint.get(1));
            packet.fieldOverlay().setStroke("blue").setFill("blue").fillCircle(lookAheadPoint.get(0), lookAheadPoint.get(1), 3);
        }





        double rot = Math.toRadians(position.get(2));
        double[] xPoints = new double[]{position.get(0) + cornerRad * Math.cos(rot + Math.PI / 4),
                position.get(0) + cornerRad * Math.cos(rot + 3 * Math.PI / 4),
                position.get(0) + cornerRad * Math.cos(rot + 5 * Math.PI / 4),
                position.get(0) + cornerRad * Math.cos(rot + 7 * Math.PI / 4)};
        double[] yPoints = new double[]{position.get(1) + cornerRad * Math.sin(rot + Math.PI / 4),
                position.get(1) + cornerRad * Math.sin(rot + 3 * Math.PI / 4),
                position.get(1) + cornerRad * Math.sin(rot + 5 * Math.PI / 4),
                position.get(1) + cornerRad * Math.sin(rot + 7 * Math.PI / 4)};


        packet.fieldOverlay().setStroke("black").strokePolygon(xPoints, yPoints);
        packet.fieldOverlay().setStroke("black").strokeLine(position.get(0), position.get(1), position.get(0) + cornerRad * Math.cos(rot + Math.PI/2), position.get(1) + cornerRad * Math.sin(rot + Math.PI/2));







        if(estimatedPosition != null) {
            double[] estXPoints = new double[]{estimatedPosition.get(0) + cornerRad * Math.cos(rot + Math.PI / 4),
                    estimatedPosition.get(0) + cornerRad * Math.cos(rot + 3 * Math.PI / 4),
                    estimatedPosition.get(0) + cornerRad * Math.cos(rot + 5 * Math.PI / 4),
                    estimatedPosition.get(0) + cornerRad * Math.cos(rot + 7 * Math.PI / 4)};
            double[] estYPoints = new double[]{estimatedPosition.get(1) + cornerRad * Math.sin(rot + Math.PI / 4),
                    estimatedPosition.get(1) + cornerRad * Math.sin(rot + 3 * Math.PI / 4),
                    estimatedPosition.get(1) + cornerRad * Math.sin(rot + 5 * Math.PI / 4),
                    estimatedPosition.get(1) + cornerRad * Math.sin(rot + 7 * Math.PI / 4)};


            packet.fieldOverlay().setStroke("red").strokePolygon(estXPoints, estYPoints).setStroke("red");
            packet.fieldOverlay().setStroke("red").strokeLine(estimatedPosition.get(0), estimatedPosition.get(1), estimatedPosition.get(0) + cornerRad * Math.cos(rot), estimatedPosition.get(0) + cornerRad * Math.sin(rot));

        }

        parent.dashboard.sendTelemetryPacket(packet);
    }
}
