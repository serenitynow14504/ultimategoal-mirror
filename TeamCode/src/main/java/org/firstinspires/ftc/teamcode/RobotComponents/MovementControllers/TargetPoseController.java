package org.firstinspires.ftc.teamcode.RobotComponents.MovementControllers;

import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Common.PIDController;
import org.firstinspires.ftc.teamcode.Common.Utilities;
import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Path;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

import java.util.HashMap;

import static java.lang.Math.sqrt;

class TargetPoseController extends MovementController {
    public TargetPoseController(Robot r, VectorD target) {
        super(r, new Path(new VectorD[] {r.getPosition(), Utilities.clipToXY(target)}));
    }

    public void followPath(double power) {
        timer = new ElapsedTime();


        PIDController r = new PIDController(RobotConstants.TRP, RobotConstants.TRI, RobotConstants.TRD);//p = 0.01   d = 0.006
        r.setSetpoint(robot.getTargetRotation());
        r.setOutputRange(0, power);  //-power to power?
        r.enable();


        double distToEnd, rampDownDist = -1, rampDownTo = 0.35;
        double closestPathPointParameter;
        timer.reset();
        do {
            VectorD pos = robot.getPose();

            path.calculateClosestData(pos);

            closestPathPointParameter = path.closestParameter();

            double lookAhead = RobotConstants.lookAheadConstant/(path.getClosestDist()/64.0 + 1.0);

            setProgress(closestPathPointParameter/path.getPathLength());

            lookAhead += closestPathPointParameter;

            VectorD globalFollowVector = path.pointFromParameter(lookAhead);

            globalFollowVector.subtract(Utilities.clipToXY(pos));



            //Obstacle Repulsion Vector
            VectorD obstacleRepel = robot.getEnvironmentRepulsionVectorFromCenter();

            double antiPathCorrect = 1;
            if(obstacleRepel.magnitude() > 1) {
                antiPathCorrect = 1/ sqrt(obstacleRepel.magnitude());
            }


            //Return to Path Correction Vector
            VectorD toNearestPoint;
            toNearestPoint = path.pointFromParameter(closestPathPointParameter).subtracted(Utilities.clipToXY(pos));


            VectorD returnToPath = new VectorD(0, 0);
            if(toNearestPoint.magnitude() != 0) {
                returnToPath = Utilities.setMagnitude(toNearestPoint,
                         -path.getLeftRight() * antiPathCorrect);
                //Utilities.log(-path.getLeftRight());
            }



            distToEnd = Utilities.distance(path.getLastPoint(), pos);

            if(lookAhead >= path.getPathLength() - 5 && rampDownDist == -1) {
                rampDownDist = distToEnd;
            }

            double rampDownCoeff = 1;
            if(rampDownDist != -1) {
                rampDownCoeff = (1-rampDownTo)*distToEnd/rampDownDist + rampDownTo;
            }



            double rCorrect = r.performPID(pos.get(2));//ny



            VectorD finalGlobalVector = globalFollowVector.added(returnToPath).added(obstacleRepel);
            finalGlobalVector = Utilities.setMagnitude(finalGlobalVector, power * rampDownCoeff);

            robot.driveTrain.setScaledPowersFromGlobalVector(finalGlobalVector, rCorrect);

            robot.displayDash(path, globalFollowVector.added(Utilities.clipToXY(pos)),
                    globalFollowVector, returnToPath, obstacleRepel, finalGlobalVector,
                    new HashMap<String, Double>() {
                        {
                            put("path normal error", path.getNormalError());
                        }
                    });


            if(robot.driveTrain.isInterrupted()) {
                robot.driveTrain.resetInterrupt();
                break;
            }

            /*if(timer.milliseconds() > 10000) {
                robot.getMyOpMode().stop();
            }*/
        } while(robot.getMyOpMode().opModeIsActive() && distToEnd > 1 && closestPathPointParameter < path.getPathLength());

        robot.driveTrain.setPowers(0);
        setProgress(0);
    }
}
