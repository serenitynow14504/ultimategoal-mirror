package org.firstinspires.ftc.teamcode.robotComponents;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.common.Polygon;
import org.firstinspires.ftc.teamcode.common.Util;
import org.firstinspires.ftc.teamcode.common.VectorD;
import org.firstinspires.ftc.teamcode.robotComponents.CV.Pipelines.ElementCV;
import org.firstinspires.ftc.teamcode.robotComponents.CV.TFOD;
import org.firstinspires.ftc.teamcode.robotComponents.CV.VuforiaRobotLocalizer;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robotComponents.PathPlanning.Environment;
import org.firstinspires.ftc.teamcode.robotComponents.PathPlanning.Path;
import org.jetbrains.annotations.NotNull;
import org.json.JSONArray;
import org.json.JSONObject;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.revextensions2.ExpansionHubEx;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.common.Util.clipToXY;
import static org.firstinspires.ftc.teamcode.common.Util.duplicate;
import static org.firstinspires.ftc.teamcode.common.Util.rotate;

public class Robot extends Thread {
    private ExpansionHubEx hub1, hub2;
    private Collection<Capability> capabilities;
    public DriveTrain driveTrain;
    public OdometryLocalizer odometry;
    public Intake intake;
    public Lift lift;
    public CarouselWheel carouselWheel;

    private TFOD tfod;
    private VuforiaRobotLocalizer vuforia;
    private OpenCvWebcam camera;
    private final ElementCV pipeline = new ElementCV();


    private final ElapsedTime runtime = new ElapsedTime();
    private LinearOpMode myOpMode;

    private volatile VectorD pose;
    private double targetRotation;
    private double rotationOffset;

    private Polygon robotShape;
    private Environment field;

    private final ArrayList<VectorD> breadCrumbs = new ArrayList<>();
    private final ElapsedTime breadCrumbTimer = new ElapsedTime();

    private final double width = RobotConstants.width;
    private final double length = RobotConstants.length;

    private int barcodeState = 0;
    private boolean vufLocInited = false;

    FtcDashboard dashboard = null;

    RobotConstants.ALLIANCES alliance;

    VectorD aimPos = FieldConstants.HIGH_GOAL;
    private boolean autoAim = false;

    private final HashMap<String, Path> PATHS = new HashMap<>();


    public enum TaskState {
        IDLE
    } // to keep track of which task the robot is currently doing

    public enum VisionMethod {
        NONE,
        VUFORIA,
        OPENCV,
        TENSORFLOW
    }



    private TaskState taskState = TaskState.IDLE;

    public void setTaskState(TaskState ts) {
        taskState = ts;
    }

    public TaskState getTaskState() {return taskState;}

    public RobotConstants.ALLIANCES getAlliance() {
        return alliance;
    }

    //define pos using bottom left corner
    public Robot(LinearOpMode op, RobotConstants.ALLIANCES alliance, Environment environment,
                 double x, double y, float r) {
        initMembers(op, alliance, environment, x + width / 2, y + length / 2, r);
    }

    //define pos using center
    public Robot(LinearOpMode op, RobotConstants.ALLIANCES alliance, Environment environment,
                 double x, double y, double r, boolean noadjust) {
        initMembers(op, alliance, environment, x, y, r);
    }

    public Robot(LinearOpMode op, RobotConstants.ALLIANCES alliance, Environment environment) {
        VectorD readPose = readPose();
        initMembers(op, alliance, environment, readPose.getX(), readPose.getY(),
                readPose.getZ());
    }

    private void initMembers(LinearOpMode op, RobotConstants.ALLIANCES alliance, Environment environment,
                             double x, double y, double r) {
        myOpMode = op;
        capabilities = new ArrayList<>();
        driveTrain = new DriveTrain(this);
        capabilities.add(driveTrain);
        odometry = new TwoWheelOdometryOld(this);
        capabilities.add(odometry);
        runtime.reset();
        if(alliance == RobotConstants.ALLIANCES.BLUE) {
            setPose(-x, y, r);
        } else {
            setPose(-x, y, r);
        }
        field = environment;
        targetRotation = pose.get(2);
        rotationOffset = targetRotation;
        this.alliance = alliance;

        intake = new Intake(this);
        capabilities.add(intake);
        lift = new Lift(this);
        capabilities.add(lift);
        carouselWheel = new CarouselWheel(this);
        capabilities.add(carouselWheel);
        tfod = new TFOD();
        vuforia = new VuforiaRobotLocalizer();

        try {

            File pathFile = new File("paths.json");
            StringBuilder output = new StringBuilder();
            FileReader fileReader = new FileReader(pathFile.getAbsolutePath());
            BufferedReader bufferedReader = new BufferedReader(fileReader);
            String line;
            while ((line = bufferedReader.readLine()) != null) {
                output.append(line).append("\n");
            }
            String response = output.toString();

            bufferedReader.close();

            JSONArray jsonArray = new JSONArray(response);

            for (int i = 0; i < jsonArray.length(); i++) {
                JSONObject path = jsonArray.getJSONObject(i);
                JSONArray jsonTransPts = path.getJSONArray("trans");
                JSONArray jsonRotPts = path.getJSONArray("rot");

                VectorD[] transPts = new VectorD[jsonTransPts.length()];
                for (int j = 0; j < transPts.length; j++) {
                    JSONArray coord = jsonTransPts.getJSONArray(j);
                    transPts[j] = new VectorD(coord.getDouble(0), coord.getDouble(1));
                }
                VectorD[] rotPts = new VectorD[jsonRotPts.length()];
                for (int j = 0; j < rotPts.length; j++) {
                    JSONArray coord = jsonRotPts.getJSONArray(j);
                    rotPts[j] = new VectorD(coord.getDouble(0), coord.getDouble(1));
                }

                PATHS.put(path.getString("name"), new Path(transPts, rotPts));
            }
        } catch (Exception e) {
            Util.log(e.toString());
        }
    }

    public void init(HardwareMap hardwareMap, VisionMethod vm) {
        init(hardwareMap);

        switch(vm) {
            case OPENCV:
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

                camera.setPipeline(pipeline);



                camera.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
                camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
                {
                    @Override
                    public void onOpened()
                    {
                        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                        camera.showFpsMeterOnViewport(false);
                    }

                    @Override
                    public void onError(int errorCode)
                    { }
                });
                break;
            case VUFORIA:
                vuforia.init(hardwareMap);
                vufLocInited = true;
                break;
            case TENSORFLOW:
                tfod.init(hardwareMap);
                break;
        }
    }

    public void init(HardwareMap hardwareMap) {
        while (dashboard == null) {
            FtcDashboard.start(null);
            dashboard=FtcDashboard.getInstance();
        }
        getMyOpMode().telemetry = dashboard.getTelemetry();

        hub1 = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        setLedColors(128, 0, 0);
        Util.log("hubs init");

        for(Capability c : capabilities)
            c.init(hardwareMap);


        vufLocInited = false;

        if(dashboard != null) {
            TelemetryPacket packet = new TelemetryPacket();

            field.show(packet, "red");
            this.show(packet);

            dashboard.sendTelemetryPacket(packet);
        }

        setLedColors(0, 0, 255);
        Util.log("init");
    }


    @Override
    public void run() {
        breadCrumbTimer.reset();

        for (Capability c : capabilities) {
            c.tickInit();
        }

        setLedColors(0, 255, 0);
        Util.log("begun");

        while(myOpMode.opModeIsActive() && !myOpMode.isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            for (Capability c : capabilities) {
                c.tick(packet);
            }

            field.show(packet, "red");
            showBreadCrumbs(packet, "brown");
            this.show(packet);
            if(breadCrumbTimer.milliseconds() > 200) {
                breadCrumbs.add(Util.addZ(Util.rotate(getPose(), -Math.PI/2).added(new VectorD(-72,
                        24)), getPose().getZ()));
                breadCrumbTimer.reset();
            }

            packet.addLine("Position: (" + getPose().getX() + ", " + getPose().getY() + ")");
            packet.put("battery voltage", getBatteryVoltage());


            dashboard.sendTelemetryPacket(packet);
        }
    }


    public void teleOp(@NotNull Gamepad gamepad1, Gamepad gamepad2) {
        for(Capability c : capabilities)
            c.teleOp(gamepad1, gamepad2);


//        driveTrain.teleOp(gamepad1, gamepad2);
//        intake.teleOp(gamepad1, gamepad2);
//        shooter.teleOp(gamepad1, gamepad2);
//        lift.teleOp(gamepad1, gamepad2);
//        carouselWheel.teleOp(gamepad1, gamepad2);

        /*if(gamepad2.right_stick_y>0.5 || gamepad2.left_trigger>=0.9) {
            stopAim();
        } else if(gamepad2.right_stick_y<-0.5) {
            aim();
        }*/

        /*if(gamepad2.left_stick_y>0.5) {
            stopAim();
        } else if(gamepad2.left_stick_y<-0.5) {
            aim();
        }*/

        /*if(gamepad1.left_stick_button) {
            odometry.pause();
        } else {
            odometry.play();
        }

        if(gamepad1.left_stick_button) {
            VectorD vuPose = getVuforiaPose();
            if(vuPose != null) {
                odometry.setHeading(vuPose.getZ());
                setPose(vuPose);
                setLedColors(0, 255, 0);
            } else {
                setLedColors(255, 0, 255);
            }
        }

        if(gamepad2.a) {
            setAimPos(FieldConstants.HIGH_GOAL);
        } else if(gamepad2.x) {
            setAimPos(FieldConstants.LEFT_POWER_SHOT);
        } else if(gamepad2.y) {
            setAimPos(FieldConstants.MID_POWER_SHOT);
        } else if(gamepad2.b) {
            setAimPos(FieldConstants.RIGHT_POWER_SHOT);
        }*/
    }

    public void debugOp(@NotNull Gamepad gamepad1, Gamepad gamepad2) {
        for(Capability c : capabilities)
            c.debugOp(gamepad1, gamepad2);
//        driveTrain.debugOp(gamepad1, gamepad2);
//        intake.debugOp(gamepad1, gamepad2);
//        shooter.debugOp(gamepad1, gamepad2);
//        lift.debugOp(gamepad1, gamepad2);

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

    public double getWidth() {return width;}
    public double getLength() {return length;}

    double getRotationOffset() {return rotationOffset;}


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
        return duplicate(pose);
    }

    public VectorD getPosition() {
        return clipToXY(pose);
    }

    public VectorD getVuforiaPose() {
        try {
            return vuforia.getPose();
        } catch(Exception e) {
            Util.log(e.getMessage());
            return null;
        }
    }

    public double getAimTargetDist() {
        return Util.distance(getPose(), aimPos);
    }

    public void setAimPos(VectorD pos) {
        aimPos = pos;
    }

    public void aim() {
        autoAim = true;
        setLedColors(255, 255, 0);
    }

    public void stopAim() {
        autoAim = false;
        setLedColors(0, 255, 0);
    }

    public boolean isAiming() {
        return autoAim;
    }

    public void setPose(VectorD p) {
        pose = Util.duplicate(p);
        updateShape(pose);
    }

    public void setPose(double x, double y, double r) {
        setPose(new VectorD(x, y, r));
    }

    public LinearOpMode getMyOpMode() {
        return myOpMode;
    }

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

    @Deprecated
    public void detectStack() {
        barcodeState = tfod.detectStack();
    }

    public void detectBarcode() {
        barcodeState = pipeline.getFinalPos();
    }

    public int getBarcodeState() {return barcodeState;}

    public double getPathProgress() {
        return driveTrain.controller.getProgress();
    }

    public void followPath(double power, long timeout, VectorD... points) {
        assert points.length > 0;
        VectorD[] withStart = new VectorD[points.length+1];
        System.arraycopy(points, 0, withStart, 1, points.length);
        withStart[0] = getPose();

        driveTrain.controller.followPath(new Path(withStart), power);
        driveTrain.controller.setTimeOut(timeout);
    }

    public void followPath(double power, VectorD... points) {
        assert points.length > 0;
        VectorD[] withStart = new VectorD[points.length+1];
        System.arraycopy(points, 0, withStart, 1, points.length);
        withStart[0] = getPose();

        driveTrain.controller.followPath(new Path(withStart), power);
        driveTrain.controller.noTimeOut();
    }

    public void posOnRobotToGlobalPos(VectorD posOnRobot, VectorD targetPos, double power,
                                      double overshoot, long timeout) {
        VectorD pos = getPosition(); //0 0
        VectorD delta = Util.setMagnitude(targetPos.subtracted(pos), Util.distance(pos,
                targetPos) + overshoot); //12 12
        VectorD end = Util.setMagnitude(delta, delta.magnitude() - posOnRobot.magnitude());//
        VectorD rotClamp = Util.setMagnitude(delta, end.magnitude() * 0.11);
        double newHeading = Math.toDegrees(Math.atan2(delta.getY(), delta.getX())) -
                Math.toDegrees(Math.atan2(posOnRobot.getY(), posOnRobot.getX()));
        VectorD[] points = {getPose(), Util.addZ(rotClamp.added(pos), newHeading), Util.addZ(end.added(pos), newHeading)};
        followPath(power, timeout, points);
    }

    public void posOnRobotToGlobalPos(VectorD posOnRobot, VectorD targetPos, double power,
                                      double overshoot) {
        posOnRobotToGlobalPos(posOnRobot, targetPos, power, overshoot,30000);
    }

    public void translate(VectorD translation, double power) {
        VectorD[] points = {getPosition(), Util.rotate(translation, getPose().getZ()).added(getPosition())};
        followPath(power, points);
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

        Util.log(String.format(Locale.getDefault(), "read pose: (%f, %f, %f)", x, y, r));
        return new VectorD(x, y, r);
    }

    //================================================================================
    //DASHBOARD TELEMETRY METHODS
    //================================================================================

    private void show(TelemetryPacket packet) {
        robotShape.show(packet, "green");
        double plusRadius = 3;

        Util.drawLocalLine(packet, new VectorD(-plusRadius, 0), new VectorD(plusRadius, 0),
                getPose(), "green");
        Util.drawLocalLine(packet, new VectorD(0, -plusRadius), new VectorD(0,
                        RobotConstants.length/2), getPose(), "green");

        //Util.drawLocalCircle(packet, RobotConstants.GRABBER_POS, 1.5, getPose(), "blue");

        //Util.drawLocalCircle(packet, RobotConstants.INTAKE_POS, 1.5, getPose(), "pink");
    }

    private void showBreadCrumbs(TelemetryPacket packet, String color) {
        for(VectorD v : breadCrumbs) {
            packet.fieldOverlay().setStroke(color).setFill(color).fillCircle(v.get(0), v.get(1), 1);
            packet.fieldOverlay().setStroke(color).setStrokeWidth(1).strokeLine(v.get(0), v.get(1),
                    v.get(0) + 2*Math.cos(Math.toRadians(v.get(2))),
                    v.get(1) + 2*Math.sin(Math.toRadians(v.get(2))));
        }
    }

    /*private void sendValues(TelemetryPacket packet, HashMap<String, Double> values) {
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
            VectorD pos = duplicate(pose);

            packet.fieldOverlay().setStroke("orange").strokeLine(pos.get(1)-72, -pos.get(0)+24,
                    lookAheadPoint.get(1)-72, -lookAheadPoint.get(0)+24);
            packet.fieldOverlay().setStroke("orange").setFill("orange").fillCircle(
                    lookAheadPoint.get(1), -lookAheadPoint.get(0), 1.5);
        }

        this.show(packet);

        drawArrow(packet, getPosition(), getPosition().added(followVec), "orange");
        drawArrow(packet, getPosition(), getPosition().added(toPathVec), "blue");
        drawArrow(packet, getPosition(), getPosition().added(repulsionVec), "red");

        drawArrow(packet, getPosition(), getPosition().added(finalVec.multiplied(12d)),
                "black");

        if(breadCrumbTimer.milliseconds() > 200) {
            breadCrumbs.add(Util.addZ(Util.rotate(getPose(), -Math.PI/2).added(new VectorD(-72,
                    24)), getPose().getZ()));
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
            breadCrumbs.add(Util.addZ(Util.rotate(getPose(), -Math.PI/2).added(new VectorD(-72,
                    24)), getPose().getZ()));
            breadCrumbTimer.reset();
        }

        sendValues(packet, valuesToGraph);

        RobotLog.d("Bruh send");
        dashboard.sendTelemetryPacket(packet);
    }


    @Deprecated
    public void displayDash(String[] telem, HashMap<String, Double> graph) {
        if(dashboard == null) return;
        TelemetryPacket packet = new TelemetryPacket();

        field.show(packet, "red");

        this.show(packet);

        if(telem != null) {
            for (String s : telem) {
                packet.addLine(s);
            }
        }
        sendValues(packet, graph);
        //breadCrumbs.add(getPosition());

        dashboard.sendTelemetryPacket(packet);
    }*/



}
