package org.firstinspires.ftc.teamcode.RobotComponents.MovementControllers;

import android.util.Pair;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Common.Line;
import org.firstinspires.ftc.teamcode.Common.PIDController;
import org.firstinspires.ftc.teamcode.Common.Utilities;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Path;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

import java.util.HashMap;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.Common.Utilities.log;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.CENTER_TO_WHEEL_DIST;


public class PurePursuitController3D extends MovementController {

    public PurePursuitController3D(Robot r, Path p) {
        super(r, p);
    }


    public void followPath(double power) {

        ElapsedTime timer = new ElapsedTime();


        /*PIDController rPID = new PIDController(RobotConstants.TRP, RobotConstants.TRI, RobotConstants.TRD);
        rPID.setSetpoint(robot.getTargetRotation());
        rPID.setOutputRange(0, power);  //-power to power?
        rPID.enable();*/


        PIDController toPathPID = new PIDController(0.01, 0, 0);
        toPathPID.setSetpoint(0);
        toPathPID.setOutputRange(0, 144);
        toPathPID.setWindowing(40);
        toPathPID.enable();

        double distToEnd, rampDownDist = -1, rampDownTo = 0.35;
        double closestPathPointParameter;
        timer.reset();
        do {
            VectorF pos = robot.getPosition();

            path.calculateClosestData(pos);

            closestPathPointParameter = path.closestParameter();

            double lookAhead = 16d;///(path.getClosestDist()/64.0 + 1.0);

            setProgress(closestPathPointParameter/path.getPathLength());

            lookAhead += closestPathPointParameter;

            VectorF lookAheadPoint = path.pointFromParameter(lookAhead);
            //VectorF globalFollowVector = lookAheadPoint.subtracted(Utilities.clipToXY(pos));
            Pair<Double, VectorF> r = turnPower(pos, lookAheadPoint, power);
            Pair<double[], VectorF> r2 = turnPower2(pos, lookAheadPoint, power);


            //Return to Path Correction
            //toNearestPoint = Utilities.clipToXY(pos).subtracted(path.pointFromParameter(closestPathPointParameter));
            double x = toPathPID.performPID(path.getNormalError()) * path.getLeftRight();
            //VECTOR 2!


            //Obstacle Repulsion Vector
            VectorF obstacleRepel = robot.getEnvironmentRepulsionVector();



            distToEnd = Utilities.distance(path.getLastPoint(), pos);

            if(lookAhead >= path.getPathLength() - 5 && rampDownDist == -1) {
                rampDownDist = distToEnd;
            }

            double rampDownCoeff = 1;
            if(rampDownDist != -1) {
                rampDownCoeff = (1-rampDownTo)*distToEnd/rampDownDist + rampDownTo;
            }




            //robot.driveTrain.setScaledPowersFromComponents(0, power*rampDownCoeff, r.first);
            robot.driveTrain.setScaledTankStrafePowers(r2.first[0]*rampDownCoeff, r2.first[1]*rampDownCoeff,
                    x);


            HashMap<String, Double> values = new HashMap<>();
            values.put("radius", Utilities.distance(pos, r.second));
            values.put("turnPower", r2.first[0]-r2.first[1]);

            log("turnPower " + (r2.first[0]-r2.first[1]));
            log("");

            robot.displayDash(path, lookAheadPoint, r.second, x, values);

            if(robot.driveTrain.isInterrupted()) {
                robot.driveTrain.resetInterrupt();
                break;
            }

        } while(robot.getMyOpMode().opModeIsActive() && distToEnd > 1 && closestPathPointParameter < path.getPathLength());

        robot.driveTrain.setPowers(0);
        setProgress(0);
    }

    private Pair<Double, VectorF> turnPower(VectorF pos, VectorF point, double forwardPower) {
        VectorF relativePoint = point.subtracted(Utilities.clipToXY(pos));
        double centralAngleRadians = Math.PI - 2d * (atan2(relativePoint.get(1),
                relativePoint.get(0))-pos.get(2));
        double curvature;
        if(centralAngleRadians == 0) {
            curvature = 0.0;
        } else {
            double radius =
                    relativePoint.magnitude() / (Math.sqrt(2d - 2d * Math.cos(centralAngleRadians)));
            curvature = 1d / abs(Math.pow(radius, 1d/4d));
            log("radius " + radius);
        }
        Line toPoint = new Line(Utilities.clipToXY(pos), point, -1);
        Line perp = toPoint.perpLine(Utilities.clipToXY(pos).added(point).multiplied(0.5f));

        Line robotSide = new Line(Utilities.clipToXY(pos), tan(Math.toRadians(pos.get(2))));

        VectorF center = perp.intersect(robotSide);


        Line forward = new Line(Utilities.clipToXY(pos), tan(Math.toRadians(pos.get(2)) + Math.PI/2));
        curvature *= -forward.calculateLeftRight(point);

        curvature *= forwardPower;


        /*Utilities.log(Utilities.distance(Utilities.clipToXY(pos), center));
        Utilities.log(Utilities.distance(point, center));
        Utilities.log("");*/
        //Utilities.log(curvature);




        return new Pair<>(curvature, center);
    }


    private Pair<double[], VectorF> turnPower2(VectorF pos, VectorF point, double forwardPower) {
        VectorF relativePoint = point.subtracted(Utilities.clipToXY(pos));
        double centralAngleRadians = abs(Math.PI - 2d * (atan2(relativePoint.get(1),
                relativePoint.get(0))-pos.get(2)));


        Line toPoint = new Line(Utilities.clipToXY(pos), point, -1);
        Line perp = toPoint.perpLine(Utilities.clipToXY(pos).added(point).multiplied(0.5f));

        Line robotSide = new Line(Utilities.clipToXY(pos), tan(Math.toRadians(pos.get(2))));

        VectorF center = perp.intersect(robotSide);


        double rot = toRadians(pos.get(2));
        VectorF forwardPoint = new VectorF(pos.get(0) + 20f * (float)cos(rot + Math.PI/2),
                pos.get(1) + 20f * (float)sin(rot + Math.PI/2));
        Line forward = new Line(Utilities.clipToXY(pos), forwardPoint, 0);

        double radius, lPow, rPow;
        int leftRight = -forward.calculateLeftRight(Utilities.clipToXY(point));
        log("leftright = " + leftRight);

        if(centralAngleRadians == 0) {
            lPow = forwardPower;
            rPow = forwardPower;
        } else {
            radius = relativePoint.magnitude() / (Math.sqrt(2d - 2d * Math.cos(centralAngleRadians)));
            log("radius " + radius);

            double magnifyConstant = 1.4;
            lPow = (radius + CENTER_TO_WHEEL_DIST*leftRight*magnifyConstant) * centralAngleRadians;
            rPow = (radius - CENTER_TO_WHEEL_DIST*leftRight*magnifyConstant) * centralAngleRadians;

            double cPow = (lPow+rPow)/2;
            double scaleFactor = forwardPower/cPow;

            lPow *= scaleFactor;
            rPow *= scaleFactor;
        }
        /*Utilities.log(Utilities.distance(Utilities.clipToXY(pos), center));
        Utilities.log(Utilities.distance(point, center));
        Utilities.log("");*/
        //Utilities.log(curvature);

        return new Pair<>(new double[] {lPow, rPow}, center);
    }

}
