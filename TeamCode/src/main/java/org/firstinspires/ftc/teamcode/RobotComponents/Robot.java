package org.firstinspires.ftc.teamcode.RobotComponents;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Common.Polygon;
import org.firstinspires.ftc.teamcode.Common.Utilities;
import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.CV.TFOD;
import org.firstinspires.ftc.teamcode.RobotComponents.CV.VuforiaRobotLocalizer;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.MovementControllers.MovementController;
import org.firstinspires.ftc.teamcode.RobotComponents.MovementControllers.PurePursuitController2D;
import org.firstinspires.ftc.teamcode.RobotComponents.MovementControllers.PurePursuitController3D;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Environment;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Path;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.RotPath;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.SplinePath;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.Common.Utilities.clipToXY;
import static org.firstinspires.ftc.teamcode.Common.Utilities.distance;
import static org.firstinspires.ftc.teamcode.Common.Utilities.drawArrow;
import static org.firstinspires.ftc.teamcode.Common.Utilities.drawLocalArrow;
import static org.firstinspires.ftc.teamcode.Common.Utilities.duplicate;
import static org.firstinspires.ftc.teamcode.Common.Utilities.rotate;

public class Robot {
    ExpansionHubEx hub1;
    ExpansionHubEx hub2;
    public DriveTrain driveTrain;
    public OdometryLocalizer odometry;
    public Imu imu;
    public Intake intake;
    public Shooter shooter;
    public Lift lift;
    public WobbleArm wobbleArm;
    private TFOD tfod;
    public VuforiaRobotLocalizer vuforia;

    private MovementController controller;

    private ElapsedTime runtime;
    private LinearOpMode myOpMode;

    private volatile VectorD position;
    private double targetRotation;
    private double rotationOffset;
    private int front;
    private int side;

    private Polygon robotShape;
    private Environment field;

    private ArrayList<VectorD> breadCrumbs = new ArrayList<>();
    private ElapsedTime breadCrumbTimer = new ElapsedTime();

    private final double width = RobotConstants.width;
    private final double length = RobotConstants.length;

    private int stackState = 0;
    private boolean vufLocInited = false;

    FtcDashboard dashboard = null;

    RobotConstants.ALLIANCES alliance;

    VectorD aimPos = FieldConstants.HIGH_GOAL;
    boolean autoAim = false;


    public enum TaskState {
        IDLE
    } // to keep track of which task the robot is currently doing



    private TaskState taskState = TaskState.IDLE;

    public void setTaskState(TaskState ts) {
        taskState = ts;
    }

    public TaskState getTaskState() {return taskState;}

    public RobotConstants.ALLIANCES getAlliance() {
        return alliance;
    }


    public Robot(LinearOpMode op, RobotConstants.ALLIANCES alliance, Environment environment,
                 double x, double y, float r) {
        initMembers(op, alliance, environment, 1, 1, x, y, r);
    }

    public Robot(LinearOpMode op, RobotConstants.ALLIANCES alliance, Environment environment,
                 double x, double y, double r, boolean noadjust) {
        initMembers(op, alliance, environment, 1, 1, x - width / 2, y - length / 2, r);
    }

    public Robot(LinearOpMode op, RobotConstants.ALLIANCES alliance, Environment environment) {
        VectorD readPose = readPose();
        initMembers(op, alliance, environment, 1, 1, readPose.getX(), readPose.getY(),
                readPose.getZ());
    }

    private void initMembers(LinearOpMode op, RobotConstants.ALLIANCES alliance, Environment environment,
                             int front,
                             int side, double x,
                             double y, double r) {
        //front: 1 = grabber side, -1 = foundation side
        //side: 1 = red side (+strafe = left if grabber is forward), -1 = blue side (+strafe = right if grabber is forward)
        // UPDATED side: 1 = normal (+strafe = right if grabber is forward), -1 = reversed (+strafe = left if grabber is forward)
        myOpMode = op;
        driveTrain = new DriveTrain(this);
        odometry = new OdometryLocalizer(this, 0, 0);
        runtime = new ElapsedTime();
        if(alliance == RobotConstants.ALLIANCES.BLUE) {
            position = new VectorD(-x - width / 2, y + length / 2, r);
        } else {
            position = new VectorD(x + width / 2, y + length / 2, r);
        }
        updateShape(position);
        field = environment;
        targetRotation = position.get(2);
        rotationOffset = targetRotation;
        this.alliance = alliance;
        this.front = front;
        this.side = side;

        //taskMachine = new TaskMachine(this, driveTrain, new TaskMap[] {});
        intake = new Intake(this);
        shooter = new Shooter(this);
        lift = new Lift(this);
        wobbleArm = new WobbleArm(this);
        tfod = new TFOD();
        vuforia = new VuforiaRobotLocalizer();
    }

    public double getWidth() {return width;}
    public double getLength() {return length;}

    double getRotationOffset() {return rotationOffset;}

    void switchDir(int dir) { //true = grabber forward
        if(dir == 1) {
            front = 1;
            side = -1;
        } else if (dir == -1){
            front = -1;
            side = 1;
        } else if (dir == 0) {
            front *= -1;
            side *= -1;
        }
    }

    public double getBatteryVoltage() {
        return hub1.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
    }

    public double getTargetRotation() {
        return targetRotation;
    }

    public void setTargetRotation(double target) {
        targetRotation = target;
    }

    public double getRuntime() {
        return runtime.milliseconds();
    }

    public VectorD getPose() {
        return duplicate(position);
    }

    public VectorD getPosition() {
        return clipToXY(position);
    }

    public VectorD getVuforiaPose() throws Exception {
        return vuforia.getPose();
    }

    public double getAimTargetDist() {
        try {
            return (vufLocInited) ? Utilities.distance(getVuforiaPose(), aimPos) : 84;
        } catch(Exception e) {
            return 84;
        }
    }


    public void setAimPos(VectorD pos) {
        aimPos = pos;
    }

    public void aim() {
        autoAim = true;
    }

    public void stopAim() {
        autoAim = false;
    }

    public void setPosition(VectorD p) {
        position = p;
        updateShape(position);
    }


    public LinearOpMode getMyOpMode() {
        return myOpMode;
    }

    int getFront() {
        return front;
    }

    int getSide() { return side; }

    public void setLedColors(int r, int g, int b) {
        hub1.setLedColor(r, g, b);
        hub2.setLedColor(r, g, b);
    }

    private void updateShape(VectorD pos) {
        double rot = Math.toRadians(pos.getZ());

        double x = RobotConstants.width/2;
        double y = RobotConstants.length/2;

        VectorD c1  = rotate(new VectorD(x, y), rot).added(clipToXY(pos));
        VectorD c2  = rotate(new VectorD(-x, y), rot).added(clipToXY(pos));
        VectorD c3  = rotate(new VectorD(-x, -y), rot).added(clipToXY(pos));
        VectorD c4  = rotate(new VectorD(x, -y), rot).added(clipToXY(pos));

        VectorD[] corners = new VectorD[] {c1, c2, c3, c4};

        robotShape = new Polygon(corners);
    }

    public VectorD getEnvironmentRepulsionVector() {
        return field.repel(robotShape);
    }

    public VectorD getEnvironmentRepulsionVectorFromCenter() {
        return field.repel(getPosition());
    }


    public void INIT(HardwareMap hardwareMap, boolean useVuf) {
        while (dashboard == null) {
            FtcDashboard.start();
            dashboard=FtcDashboard.getInstance();
        }
        getMyOpMode().telemetry = dashboard.getTelemetry();

        hub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        setLedColors(128, 0, 0);
        Utilities.log("hubs init");

        imu = new Imu(myOpMode.hardwareMap.get(BNO055IMU.class, "imu"), this);
        Utilities.log("imu init");
        driveTrain.init(hardwareMap);
        Utilities.log("dt init");
        odometry.init(hardwareMap);
        Utilities.log("odo init");
        intake.init(hardwareMap);
        Utilities.log("intake init");
        shooter.init(hardwareMap);
        Utilities.log("shoot init");
        lift.init(hardwareMap);
        Utilities.log("lift init");
        wobbleArm.init(hardwareMap);
        Utilities.log("arm init");

        if(useVuf) {
            vuforia.init(hardwareMap);
        } else {
            tfod.init(hardwareMap);
        }
        vufLocInited = useVuf;

        //

        displayDash(null);

        setLedColors(0, 0, 255);
        Utilities.log("init");
    }

    public void begin() {
        breadCrumbTimer.reset();

        odometry.start();
        driveTrain.start();
        intake.start();
        shooter.start();
        lift.start();

        setLedColors(0, 255, 0);
        Utilities.log("begun");
    }



    public void teleOp(@NotNull Gamepad gamepad1, Gamepad gamepad2) {
        driveTrain.teleOp(gamepad1, gamepad2);
        intake.teleOp(gamepad1, gamepad2);
        shooter.teleOp(gamepad1, gamepad2);
        wobbleArm.teleOp(gamepad1, gamepad2);
        lift.teleOp(gamepad1, gamepad2);

        if(gamepad2.right_stick_y>0.5) {
            stopAim();
        } else if(gamepad2.right_stick_y<-0.5) {
            aim();
        }

        if(gamepad2.left_stick_y>0.5) {
            stopAim();
        } else if(gamepad2.left_stick_y<-0.5) {
            aim();
        }

        if(gamepad2.a) {
            setAimPos(FieldConstants.HIGH_GOAL);
        } else if(gamepad2.x) {
            setAimPos(FieldConstants.LEFT_POWER_SHOT);
        } else if(gamepad2.y) {
            setAimPos(FieldConstants.MID_POWER_SHOT);
        } else if(gamepad2.b) {
            setAimPos(FieldConstants.RIGHT_POWER_SHOT);
        }

        //getMyOpMode().telemetry.update();
        RobotLog.d("Bruh battery: " + getBatteryVoltage());
    }

    public void debugOp(@NotNull Gamepad gamepad1, Gamepad gamepad2) {
        driveTrain.debugOp(gamepad1, gamepad2);
        intake.debugOp(gamepad1, gamepad2);
        shooter.debugOp(gamepad1, gamepad2);
        wobbleArm.debugOp(gamepad1, gamepad2);
        lift.debugOp(gamepad1, gamepad2);

        stopAim();

        if(gamepad2.a) {
            setAimPos(FieldConstants.HIGH_GOAL);
        } else if(gamepad2.x) {
            setAimPos(FieldConstants.LEFT_POWER_SHOT);
        } else if(gamepad2.y) {
            setAimPos(FieldConstants.MID_POWER_SHOT);
        } else if(gamepad2.b) {
            setAimPos(FieldConstants.RIGHT_POWER_SHOT);
        }

        //getMyOpMode().telemetry.update();
        RobotLog.d("Bruh battery: " + getBatteryVoltage());
    }

    public void detectStack() {
        stackState = tfod.detectStack();
    }

    public int getStackState() {return stackState;}

    public double getPathProgress() {
        return controller.getProgress();
    }


    public void followPath2D(VectorD[] points, double power) {
        Path path;
        if(points[0].hasZ()) {
            path = new RotPath(points);
        } else {
            path = new Path(points);
        }
        controller = new PurePursuitController2D(this, path);
        controller.followPath(power);
    }

    public void followSplinedPath2D(VectorD[] points, double power) {
        controller = new PurePursuitController2D(this, new SplinePath(points));
        controller.followPath(power);
    }

    /*public void followPath3D(VectorD[] points, double power) {
        PurePursuitController3D controller = new PurePursuitController3D(this, new Path(points));
        controller.followPath(power);
    }*/

    public void followSplinedPath3D(VectorD[] points, double power) {
        controller = new PurePursuitController3D(this, new SplinePath(points));
        controller.followPath(power);
    }

    public void posOnRobotToGlobalPos(VectorD posOnRobot, VectorD targetPos) {
        VectorD pos = getPosition();
        VectorD delta = targetPos.subtracted(pos);
        VectorD end = Utilities.setMagnitude(delta, delta.magnitude()-posOnRobot.magnitude());
        VectorD rotClamp = Utilities.setMagnitude(delta, end.magnitude()*0.5);
        double newHeading = Math.toDegrees(Math.atan2(delta.getY(), delta.getX())) - Math.toDegrees(Math.atan2(posOnRobot.getY(), posOnRobot.getX()));
        VectorD[] points = {getPose(), Utilities.addZ(rotClamp.added(pos), newHeading), Utilities.addZ(end.added(pos), newHeading)};
        followPath2D(points, 0.7);
    }

    public void translate(VectorD translation, double power) {
        VectorD[] points = {getPosition(), Utilities.rotate(translation, -getPose().getZ()).added(getPosition())};
        followPath2D(points, power);
    }

    public void writePose() {
        File poseXFile = AppUtil.getInstance().getSettingsFile("poseX.txt");
        ReadWriteFile.writeFile(poseXFile, String.valueOf(getPose().getX()));

        File poseYFile = AppUtil.getInstance().getSettingsFile("poseY.txt");
        ReadWriteFile.writeFile(poseYFile, String.valueOf(getPose().getY()));

        File poseRFile = AppUtil.getInstance().getSettingsFile("poseR.txt");
        ReadWriteFile.writeFile(poseRFile, String.valueOf(getPose().getZ()));
    }

    public VectorD readPose() {
        File poseXFile = AppUtil.getInstance().getSettingsFile("poseX.txt");
        double x = Double.parseDouble(ReadWriteFile.readFile(poseXFile).trim());

        File poseYFile = AppUtil.getInstance().getSettingsFile("poseY.txt");
        double y = Double.parseDouble(ReadWriteFile.readFile(poseYFile).trim());

        File poseRFile = AppUtil.getInstance().getSettingsFile("poseR.txt");
        double r = Double.parseDouble(ReadWriteFile.readFile(poseRFile).trim());

        return new VectorD(x, y, r);
    }

    //================================================================================
    //DASHBOARD TELEMETRY METHODS
    //================================================================================

    private void show(TelemetryPacket packet) {
        robotShape.show(packet, "green");
        double rot = Math.toRadians(position.get(2));
        double plusRadius = 3;
        packet.fieldOverlay().setStroke("green").strokeLine(
                position.get(0) - plusRadius * Math.cos(rot + Math.PI/2),
                position.get(1) - plusRadius * Math.sin(rot + Math.PI/2),
                position.get(0) + getLength()/2 * Math.cos(rot + Math.PI/2),
                position.get(1) + getLength()/2 * Math.sin(rot + Math.PI/2));
        packet.fieldOverlay().setStroke("green").strokeLine(
                position.get(0) - plusRadius * Math.cos(rot),
                position.get(1) - plusRadius * Math.sin(rot),
                position.get(0) + plusRadius * Math.cos(rot),
                position.get(1) + plusRadius * Math.sin(rot));
        //packet.fieldOverlay().setFill("red").fillCircle(position.get(0), position.get(1), 4);
        //RobotLog.d("Bruh show()" + position);
    }

    private void showBreadCrumbs(TelemetryPacket packet, String color) {
        for(VectorD v : breadCrumbs) {
            packet.fieldOverlay().setStroke(color).setFill(color).fillCircle(v.get(0), v.get(1), 1);
            packet.fieldOverlay().setStroke(color).setStrokeWidth(1).strokeLine(v.get(0), v.get(1),
                    v.get(0) + 2*Math.cos(Math.toRadians(v.get(2))+Math.PI/2),
                    v.get(1) + 2*Math.sin(Math.toRadians(v.get(2))+Math.PI/2));
        }
    }

    private void sendValues(TelemetryPacket packet, HashMap<String, Double> values) {
        for (Map.Entry d : values.entrySet()) {
            packet.put((String) d.getKey(), d.getValue());
        }
    }

    public void displayDash(Path path, VectorD lookAheadPoint, VectorD followVec,
                            VectorD toPathVec, VectorD repulsionVec, VectorD finalVec,
                            HashMap<String, Double> valuesToGraph) {
        if(dashboard == null) return;
        TelemetryPacket packet = new TelemetryPacket();

        field.show(packet, "red");

        if(path != null) {
            path.show(packet, "black");
        }

        showBreadCrumbs(packet, "brown");

        if(lookAheadPoint != null) {
            VectorD pos = position;

            packet.fieldOverlay().setStroke("orange").strokeLine(pos.get(0), pos.get(1),
                    lookAheadPoint.get(0), lookAheadPoint.get(1));
            packet.fieldOverlay().setStroke("orange").setFill("orange").fillCircle(
                    lookAheadPoint.get(0), lookAheadPoint.get(1), 1.5);
        }

        this.show(packet);

        drawArrow(packet, getPosition(), getPosition().added(followVec), "orange");
        drawArrow(packet, getPosition(), getPosition().added(toPathVec), "blue");
        drawArrow(packet, getPosition(), getPosition().added(repulsionVec), "red");

        drawArrow(packet, getPosition(), getPosition().added(finalVec.multiplied(12d)),
                "black");

        if(breadCrumbTimer.milliseconds() > 200) {
            breadCrumbs.add(getPose());
            breadCrumbTimer.reset();
        }

        sendValues(packet, valuesToGraph);

        dashboard.sendTelemetryPacket(packet);
    }

    public void displayDash(Path path, @NonNull VectorD lookAheadPoint, VectorD arcCenter,
                            double toPath, HashMap<String, Double> valuesToGraph) {
        if(dashboard == null) return;
        TelemetryPacket packet = new TelemetryPacket();

        field.show(packet, "red");

        if(path != null) {
            path.show(packet, "black");
        }

        showBreadCrumbs(packet, "brown");

        VectorD pos = position;

        //packet.fieldOverlay().setStroke("orange").strokeLine(pos.get(0), pos.get(1),
        //        lookAheadPoint.get(0), lookAheadPoint.get(1));
        packet.fieldOverlay().setStroke("orange").setFill("orange").fillCircle(
                lookAheadPoint.get(0), lookAheadPoint.get(1), 1.5);


        this.show(packet);

        //Utilities.drawArrow(packet, get2DPosition(), get2DPosition().added(toPathVec), "blue");
        drawLocalArrow(packet, new VectorD(0, 0), new VectorD(-(float)toPath*12, 0),
                getPose(), "blue");
        if(arcCenter == null) {
            packet.fieldOverlay().setStroke("orange").strokeLine(getPosition().get(0),
                    getPosition().get(1), lookAheadPoint.get(0), lookAheadPoint.get(1));
        } else {
            //Utilities.drawArc(packet, get2DPosition(), lookAheadPoint, arcCenter, 10, "orange");
            packet.fieldOverlay().setStroke("orange").strokeCircle(arcCenter.get(0),
                    arcCenter.get(1), distance(arcCenter, lookAheadPoint));
            packet.fieldOverlay().setStroke("orange").strokeCircle(arcCenter.get(0),
                    arcCenter.get(1), 2);
        }


        if(breadCrumbTimer.milliseconds() > 200) {
            breadCrumbs.add(getPose());
            breadCrumbTimer.reset();
        }

        sendValues(packet, valuesToGraph);

        RobotLog.d("Bruh send");
        dashboard.sendTelemetryPacket(packet);
    }


    public void displayDash(Path path, VectorD lookAheadPoint) {
        if(dashboard == null) return;
        TelemetryPacket packet = new TelemetryPacket();

        if(path != null) {
            path.show(packet, "black");
        }

        showBreadCrumbs(packet, "brown");

        if(lookAheadPoint != null) {
            VectorD pos = position;

            packet.fieldOverlay().setStroke("orange").strokeLine(pos.get(0), pos.get(1),
                    lookAheadPoint.get(0), lookAheadPoint.get(1));
            packet.fieldOverlay().setStroke("orange").setFill("orange").fillCircle(
                    lookAheadPoint.get(0), lookAheadPoint.get(1), 1.5);
        }

        this.show(packet);

        if(breadCrumbTimer.milliseconds() > 200) {
            breadCrumbs.add(getPose());
            breadCrumbTimer.reset();
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void displayDash(String[] telem) {
        if(dashboard == null) return;
        TelemetryPacket packet = new TelemetryPacket();

        field.show(packet, "red");

        this.show(packet);

        if(telem != null) {
            for (String s : telem) {
                packet.addLine(s);
            }
        }

        //breadCrumbs.add(getPosition());

        dashboard.sendTelemetryPacket(packet);
    }

}
