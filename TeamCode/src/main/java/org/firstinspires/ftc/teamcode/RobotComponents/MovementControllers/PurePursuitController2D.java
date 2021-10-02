package org.firstinspires.ftc.teamcode.RobotComponents.MovementControllers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Line;
import org.firstinspires.ftc.teamcode.Common.PIDController;
import org.firstinspires.ftc.teamcode.Common.Util;
import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Path;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.RotPath;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.HashMap;

import static java.lang.Math.sqrt;

//@Config
public class PurePursuitController2D extends MovementController {

    public PurePursuitController2D(Robot r, Path p) {
        super(r, p);
    }

    public static double TpK_P = 2.2;
    public static double TpK_I = 0;
    public static double TpK_D = 2.2;
    //previous working set: 3, 0, 1.5

    public static double minStrafeCorrectCoeff = 0.6;
    private Line strafeCorrectMap = new Line(new VectorD(0, minStrafeCorrectCoeff), new VectorD(1, 1));
    public void followPath(double power, long timeOutMillis) {

        timer = new ElapsedTime();


        PIDController r = new PIDController(RobotConstants.TRP, RobotConstants.TRI, RobotConstants.TRD);//p = 0.01   d = 0.006
        r.setSetpoint(robot.getTargetRotation());
        r.setInputRange(-180, 180);
        r.setOutputRange(0, power);  //-power to power?
        r.setAbsoluteTolerance(1);
        r.enable();


        PIDController toPath = new PIDController(TpK_P, TpK_I, TpK_D);
        toPath.setSetpoint(0);
        toPath.setOutputRange(0, 144);
        toPath.setWindowing(40);
        toPath.enable();

        double distToEnd, rampDownDist = -1, rampDownTo = 0.15;
        double closestPathPointParameter;
        timer.reset();
        boolean done;
        do {
            VectorD pos = robot.getPose();

            path.calculateClosestData(pos);

            closestPathPointParameter = path.closestParameter();

            double lookAhead = RobotConstants.lookAheadConstant/(path.getClosestDist()/64.0 + 1.0);

            setProgress(closestPathPointParameter/path.getPathLength());

            lookAhead += closestPathPointParameter;

            VectorD globalFollowVector = path.pointFromParameter(lookAhead);

            globalFollowVector.subtract(Util.clipToXY(pos));




            //Obstacle Repulsion Vector
            VectorD obstacleRepel = robot.getEnvironmentRepulsionVectorFromCenter();

            double antiPathCorrect = 1;
            if(obstacleRepel.magnitude() > 1) {
                antiPathCorrect = 1/ sqrt(obstacleRepel.magnitude());
            }



            //Return to Path Correction Vector
            VectorD toNearestPoint;
            toNearestPoint = path.pointFromParameter(closestPathPointParameter).subtracted(Util.clipToXY(pos));

            double strafeAngleCorrect = -1;
            VectorD returnToPath = new VectorD(0, 0);
            final double toPathPIDOutput = toPath.performPID(path.getNormalError());
            if(toNearestPoint.magnitude() != 0) {
                returnToPath = Util.setMagnitude(toNearestPoint,
                        toPathPIDOutput * -path.getLeftRight() * antiPathCorrect);
                //Utilities.log(-path.getLeftRight());


                strafeAngleCorrect = returnToPath.dotProduct(Util.rotate(new VectorD(1, 0)
                        , Math.toRadians(pos.getZ())))/returnToPath.magnitude();
                strafeAngleCorrect = strafeCorrectMap.yFromX(Math.abs(strafeAngleCorrect));
                returnToPath.multiply(strafeAngleCorrect);
                //account for increased efficiency when toPath correcting while going sideways

            }
            final double strafeAngleCorrectLog = strafeAngleCorrect;



            distToEnd = Util.distance(path.getLastPoint(), pos);

            if(lookAhead >= path.getPathLength() - 5 && rampDownDist == -1) {
                rampDownDist = distToEnd;
            }

            double rampDownCoeff = 1;
            if(rampDownDist != -1) {
                rampDownCoeff = (1-rampDownTo)*distToEnd/rampDownDist + rampDownTo;
            }

            if(path instanceof RotPath) {
                r.setSetpoint(((RotPath) path).rotFromParam(closestPathPointParameter));
            }

            double rCorrect = r.performPID(pos.getZ());//ny


            VectorD finalGlobalVector = globalFollowVector.added(returnToPath).added(obstacleRepel);
            finalGlobalVector = Util.setMagnitude(finalGlobalVector, power * rampDownCoeff);


            //robot.driveTrain.setScaledPowersFromGlobalVector(finalGlobalVector, rCorrect);

            if(robot.isAiming()) {
                robot.driveTrain.powers.setGlobalTrans(finalGlobalVector, robot.getPose().getZ());
            } else {
                robot.driveTrain.powers.setFromGlobalVector(Util.addZ(finalGlobalVector, rCorrect),
                        robot.getPose().getZ());
            }



            //Util.log(r.onTarget() ? 1 : 0);
            //Util.log(robot.getPose().getZ());
            final boolean posDone = (distToEnd<1 || closestPathPointParameter >= path.getPathLength());
            final boolean rotDone =
                    (!(path instanceof RotPath) || r.onTarget() || robot.isAiming());
            robot.displayDash(path, globalFollowVector.added(Util.clipToXY(pos)),
                    globalFollowVector, returnToPath, obstacleRepel, finalGlobalVector,
                    new HashMap<String, Double>() {
                        {
                            put("path normal error", path.getNormalError());
                            put("PID output", toPathPIDOutput);
                            put("intake current draw: ",
                                    robot.intake.intakeWheels.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
                            put("pos done: ", (posDone) ? 1d : 0d);
                            put("rot done: ", (rotDone) ? 1d : 0d);
                            put("strafe angle correct", strafeAngleCorrectLog);
                        }
                    });


            if(robot.driveTrain.isInterrupted()) {
                robot.driveTrain.resetInterrupt();
                break;
            }

            /*if(timer.milliseconds() > 10000) {
                robot.getMyOpMode().stop();
            }*/
            done = posDone && rotDone;
        } while(robot.getMyOpMode().opModeIsActive() && !robot.getMyOpMode().isStopRequested() && !done && timer.milliseconds()<timeOutMillis);

        robot.driveTrain.powers.set(0);
        setProgress(0);
        Util.log("PATH COMPLETED");
    }

}
