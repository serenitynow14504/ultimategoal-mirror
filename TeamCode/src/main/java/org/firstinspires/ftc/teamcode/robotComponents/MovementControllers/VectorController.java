package org.firstinspires.ftc.teamcode.robotComponents.MovementControllers;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.common.Line;
import org.firstinspires.ftc.teamcode.common.PIDController;
import org.firstinspires.ftc.teamcode.common.Util;
import org.firstinspires.ftc.teamcode.common.VectorD;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Robot;

import static java.lang.Math.sqrt;
import static org.firstinspires.ftc.teamcode.common.Util.drawArrow;

//@Config
public class VectorController extends MovementController {
    public static double TpK_P = 2.2;
    public static double TpK_I = 0;
    public static double TpK_D = 2.2;
    //previous working set: 3, 0, 1.5

    public static double minStrafeCorrectCoeff = 0.6;
    private Line strafeCorrectMap = new Line(new VectorD(0, minStrafeCorrectCoeff), new VectorD(1, 1));
    private PIDController r, toPath;

    private double rampDownDist = -1;
    private final double rampDownTo = 0.15;

    public VectorController(Robot robot) {
        super(robot.driveTrain.powers, robot);

        r = new PIDController(RobotConstants.TRP, RobotConstants.TRI, RobotConstants.TRD);//p = 0.01   d = 0.006
        r.setSetpoint(robot.getTargetRotation());
        r.setInputRange(-180, 180);
        r.setOutputRange(0, power);  //-power to power?
        r.setAbsoluteTolerance(1);
        r.enable();


        toPath = new PIDController(TpK_P, TpK_I, TpK_D);
        toPath.setSetpoint(0);
        toPath.setOutputRange(0, 144);
        toPath.setWindowing(40);
        toPath.enable();
    }

    public void followPath(double power, long timeOutMillis) {



        double distToEnd;
        double closestPathPointParameter;
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

            r.setSetpoint(path.rotFromParam(closestPathPointParameter));

            double rCorrect = r.performPID(pos.getZ());//ny


            VectorD finalGlobalVector = globalFollowVector.added(returnToPath).added(obstacleRepel);
            finalGlobalVector = Util.setMagnitude(finalGlobalVector, power * rampDownCoeff);


            //robot.driveTrain.setScaledPowersFromGlobalVector(finalGlobalVector, rCorrect);

            /*if(robot.isAiming()) {
                robot.driveTrain.powers.setGlobalTrans(finalGlobalVector, robot.getPose().getZ());
            } else {

            }*/
            powers.setFromGlobalVector(Util.addZ(finalGlobalVector, rCorrect),
                    robot.getPose().getZ());


            //Util.log(r.onTarget() ? 1 : 0);
            //Util.log(robot.getPose().getZ());
            final boolean posDone = (distToEnd<1 || closestPathPointParameter >= path.getPathLength());
            final boolean rotDone = (r.onTarget() || robot.isAiming());
            /*robot.displayDash(path, globalFollowVector.added(Util.clipToXY(pos)),
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
                    });*/


            if(robot.driveTrain.isDtInterrupted()) {
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

    @Override
    protected void doTick(VectorD pose, TelemetryPacket packet) {
        path.calculateClosestData(pose);

        double closestPathPointParameter = path.closestParameter();

        double lookAhead = RobotConstants.lookAheadConstant/(path.getClosestDist()/64.0 + 1.0);

        setProgress(closestPathPointParameter/path.getPathLength());

        lookAhead += closestPathPointParameter;

        VectorD globalFollowVector = path.pointFromParameter(lookAhead);

        globalFollowVector.subtract(Util.clipToXY(pose));




        //Obstacle Repulsion Vector
        VectorD obstacleRepel = robot.getEnvironmentRepulsionVectorFromCenter();

        double antiPathCorrect = 1;
        if(obstacleRepel.magnitude() > 1) {
            antiPathCorrect = 1/ sqrt(obstacleRepel.magnitude());
        }



        //Return to Path Correction Vector
        VectorD toNearestPoint;
        toNearestPoint = path.pointFromParameter(closestPathPointParameter).subtracted(Util.clipToXY(pose));

        double strafeAngleCorrect = -1;
        VectorD returnToPath = new VectorD(0, 0);
        final double toPathPIDOutput = toPath.performPID(path.getNormalError());
        if(toNearestPoint.magnitude() != 0) {
            returnToPath = Util.setMagnitude(toNearestPoint,
                    toPathPIDOutput * -path.getLeftRight() * antiPathCorrect);
            //Utilities.log(-path.getLeftRight());


            strafeAngleCorrect = returnToPath.dotProduct(Util.rotate(new VectorD(1, 0)
                    , Math.toRadians(pose.getZ())))/returnToPath.magnitude();
            strafeAngleCorrect = strafeCorrectMap.yFromX(Math.abs(strafeAngleCorrect));
            returnToPath.multiply(strafeAngleCorrect);
            //account for increased efficiency when toPath correcting while going sideways

        }


        double distToEnd = Util.distance(path.getLastPoint(), pose);

        if(lookAhead >= path.getPathLength() - 5 && rampDownDist == -1) {
            rampDownDist = distToEnd;
        }

        double rampDownCoeff = 1;
        if(rampDownDist != -1) {
            rampDownCoeff = (1-rampDownTo)*distToEnd/rampDownDist + rampDownTo;
        }

        r.setSetpoint(path.rotFromParam(closestPathPointParameter));

        double rCorrect = r.performPID(pose.getZ());//ny


        VectorD finalGlobalVector = globalFollowVector.added(returnToPath).added(obstacleRepel);
        finalGlobalVector = Util.setMagnitude(finalGlobalVector, power * rampDownCoeff);


        //robot.driveTrain.setScaledPowersFromGlobalVector(finalGlobalVector, rCorrect);

            /*if(robot.isAiming()) {
                robot.driveTrain.powers.setGlobalTrans(finalGlobalVector, robot.getPose().getZ());
            } else {

            }*/
        powers.setFromGlobalVector(Util.addZ(finalGlobalVector, rCorrect),
                robot.getPose().getZ());



        //telemetry

        path.show(packet, "black");

        VectorD lookAheadPoint = globalFollowVector.added(Util.clipToXY(pose));
        packet.fieldOverlay().setStroke("orange").strokeLine(pose.get(1)-72, -pose.get(0)+24,
                lookAheadPoint.get(1)-72, -lookAheadPoint.get(0)+24);
        packet.fieldOverlay().setStroke("orange").setFill("orange").fillCircle(
                lookAheadPoint.get(1), -lookAheadPoint.get(0), 1.5);

        drawArrow(packet, pose, pose.added(globalFollowVector), "orange");
        drawArrow(packet, pose, pose.added(returnToPath), "blue");
        drawArrow(packet, pose, pose.added(obstacleRepel), "red");
        drawArrow(packet, pose, pose.added(finalGlobalVector.multiplied(12d)), "black");

        packet.put("path normal error", path.getNormalError());
        packet.put("PID output", toPathPIDOutput);
        packet.put("strafe angle correct", strafeAngleCorrect);

    }

    @Override
    protected void doInit() {
        rampDownDist = -1;
    }

    @Override
    protected void doFinish() {
        powers.set(0);
    }

    @Override
    protected boolean shouldExit(VectorD pose) {
        if(robot.driveTrain.isDtInterrupted()) {
            robot.driveTrain.resetInterrupt();
            return true;
        }
        path.calculateClosestData(pose);
        boolean posDone = (Util.distance(path.getLastPoint(), pose)<1 || path.closestParameter() >= path.getPathLength());
        boolean rotDone = r.onTarget();
        return (posDone && rotDone);
    }

}
