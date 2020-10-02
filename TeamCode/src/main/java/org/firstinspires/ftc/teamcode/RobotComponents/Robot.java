package org.firstinspires.ftc.teamcode.RobotComponents;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Common.Polygon;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.MovementControllers.MovementController;
import org.firstinspires.ftc.teamcode.RobotComponents.MovementControllers.PurePursuitController2D;
import org.firstinspires.ftc.teamcode.RobotComponents.MovementControllers.PurePursuitController3D;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Environment;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Path;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.SplinePath;
import org.firstinspires.ftc.teamcode.RobotComponents.Tasks.TaskMachine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.Common.Utilities.clipToXY;
import static org.firstinspires.ftc.teamcode.Common.Utilities.distance;
import static org.firstinspires.ftc.teamcode.Common.Utilities.drawArrow;
import static org.firstinspires.ftc.teamcode.Common.Utilities.drawLocalArrow;
import static org.firstinspires.ftc.teamcode.Common.Utilities.duplicate;
import static org.firstinspires.ftc.teamcode.Common.Utilities.rotate;

public class Robot {
    public DriveTrain driveTrain;
    public Lift lift;
    public Odometry odometry;
    public Latches latches;
    public Imu imu;
    public Sensors sensors;
    public Tape tape;
    private MovementController controller;

    private TaskMachine taskMachine;

    private ElapsedTime runtime;
    private LinearOpMode myOpMode;

    private volatile VectorF position, estimatedPosition;   //HAS BEEN MADE VOLATILE
    private double targetRotation;
    private double rotationOffset;
    private int front;
    private int side;

    private Polygon shape;
    private Environment field;

    private ArrayList<VectorF> breadCrumbs = new ArrayList<>();
    private ElapsedTime breadCrumbTimer = new ElapsedTime();


    private int skyStonePos = 2;   //0 - 2
    private int[] skyStoneOrder = new int[6];

    private final float width = RobotConstants.width;
    private final float length = RobotConstants.length;

    FtcDashboard dashboard = null;

    RobotConstants.ALLIANCES alliance;

    public enum TaskState {
        IDLE,
        GO_TO_STONE,
        GRAB_STONE,
        GO_TO_FOUNDATION,
        DROP_STONE,
        GO_BACK_TO_STONE,
        GRAB_FOUNDATION,
        MOVE_FOUNDATION,
        PARK
    } // to keep track of which task the robot is currently doing



    private TaskState taskState = TaskState.IDLE;

    public void setTaskState(TaskState ts) {
        taskState = ts;
    }

    public TaskState getTaskState() {return taskState;}

    public RobotConstants.ALLIANCES getAlliance() {
        return alliance;
    }


    public Robot(LinearOpMode op, RobotConstants.ALLIANCES alliance, Environment environment, int front, int side, float x, float y, float r) {
        initMembers(op, alliance, environment, front, side, x, y, r);
    }

    public Robot(LinearOpMode op, RobotConstants.ALLIANCES alliance, Environment environment, int front, int side, float x, float y, float r, boolean noadjust) {
        initMembers(op, alliance, environment, front, side, x - width / 2, y - length / 2, r);
    }

    private void initMembers(LinearOpMode op, RobotConstants.ALLIANCES alliance, Environment environment,
                             int front,
                             int side, float x,
                             float y, float r) {
        //front: 1 = grabber side, -1 = foundation side
        //side: 1 = red side (+strafe = left if grabber is forward), -1 = blue side (+strafe = right if grabber is forward)
        // UPDATED side: 1 = normal (+strafe = right if grabber is forward), -1 = reversed (+strafe = left if grabber is forward)

        myOpMode = op;
        driveTrain = new DriveTrain(this);
        lift = new Lift(this);
        odometry = new Odometry(this, 0, 0, Odometry.MODES.LINE);
        runtime = new ElapsedTime();
        latches = new Latches(this);
        if(alliance == RobotConstants.ALLIANCES.RED) {
            position = new VectorF(x + width / 2, y + length / 2, r);
        } else if(alliance == RobotConstants.ALLIANCES.BLUE) {
            position = new VectorF(-x - width / 2, y + length / 2, r);
        }
        updateShape(position);
        field = environment;
        estimatedPosition = new VectorF(position.get(0), position.get(1));
        targetRotation = position.get(2);
        rotationOffset = targetRotation;
        this.alliance = alliance;
        this.front = front;
        this.side = side;

        sensors = new Sensors(this);
        tape = new Tape(this);
        //taskMachine = new TaskMachine(this, driveTrain, new TaskMap[] {});
    }

    public void setDriveTrainPurePursuit() {
        driveTrain = new PurePursuitDrivetrain(this, odometry.getMode());
    }

    public double calculateSStonePos(int stone) {
        return 8*stone + 4;
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

    public double getTargetRotation() {
        return targetRotation;
    }

    public void setTargetRotation(double target) {
        targetRotation = target;
    }

    private void setSkyStonePos(int pos) {
        skyStonePos = pos;
    }

    public void setSkyStoneOrder(int pos) {
        setSkyStonePos(pos);

        boolean[] stones = new boolean[] {true, true, true, true, true, true};

        skyStoneOrder[0] = pos;
        if(pos == 0) {
            skyStoneOrder[0] = pos + 3;
        }
        stones[skyStoneOrder[0]] = false;

        skyStoneOrder[1] = pos + 3;
        if(pos == 0) {
            skyStoneOrder[1] = 5;
        }
        stones[skyStoneOrder[1]] = false;

        int counter = 2;
        for(int i = 5; i >= 0; i--) {
            if(stones[i]) {
                skyStoneOrder[counter] = i;
                stones[skyStoneOrder[counter]] = false;
                counter++;
            }

        }
    }

    public int getSkyStoneInOrder(int i) {return skyStoneOrder[i];}

    public int getSkyStonePos() {
        return skyStonePos;
    }

    public boolean amIBusy() {
        return (driveTrain.amIBusy() || lift.amIBusy() || latches.amIBusy());
    }

    public double getRuntime() {
        return runtime.milliseconds();
    }

    public VectorF getPosition() {
        return duplicate(position);
    }

    public VectorF get2DPosition() {
        return clipToXY(position);
    }

    public void setPosition(VectorF p) {
        position = p;
        updateShape(position);
    }

    public VectorF getEstimatedPosition() {return estimatedPosition;}

    public void setEstimatedPosition(VectorF pos) {estimatedPosition = pos;}

    public void setEstimatedPosition(float x, float y) {estimatedPosition = new VectorF(x, y);}

    public LinearOpMode getMyOpMode() {
        return myOpMode;
    }

    public int getFront() {
        return front;
    }

    public int getSide() { return side; }

    public void moveForward(double speed, double inches){
        //driveTrain.encoderDrive(speed, inches, 20);
    }

    public void strafeDrive(double speed, double inches) {
        //driveTrain.strafeDrive(speed, inches, 5);
    }

    public double inch(double cm) {return cm/2.54;}

    public void setDriveTrainPowers(double speed) {driveTrain.setPowers(speed);}

    private void updateShape(VectorF pos) {
        double rot = Math.toRadians(pos.get(2));

        float x = RobotConstants.width/2;
        float y = RobotConstants.length/2;

        VectorF c1  = rotate(new VectorF(x, y), rot).added(clipToXY(pos));
        VectorF c2  = rotate(new VectorF(-x, y), rot).added(clipToXY(pos));
        VectorF c3  = rotate(new VectorF(-x, -y), rot).added(clipToXY(pos));
        VectorF c4  = rotate(new VectorF(x, -y), rot).added(clipToXY(pos));

        VectorF[] corners = new VectorF[] {c1, c2, c3, c4};

        shape = new Polygon(corners);
    }

    public VectorF getEnvironmentRepulsionVector() {
        return field.repel(shape);
    }

    public VectorF getEnvironmentRepulsionVectorFromCenter() {
        return field.repel(get2DPosition());
    }


    public void INIT() {
        while (dashboard==null) {
            FtcDashboard.start();
            dashboard=FtcDashboard.getInstance();
        }
        getMyOpMode().telemetry = dashboard.getTelemetry();

        imu = new Imu(myOpMode.hardwareMap.get(BNO055IMU.class, "imu"), this);
        driveTrain.init();
        lift.init();
        latches.init();
        latches.latch(false);
        odometry.init();
        sensors.init();
        tape.init();

        displayDash();
    }

    public void begin() {
        lift.start();
        odometry.start();
        latches.start();
        tape.start();
        breadCrumbTimer.reset();
    }



    public void teleOp(Gamepad gamepad1, Gamepad gamepad2) {
        if(gamepad1.right_bumper) {
            switchDir(1);
        } else if(gamepad1.left_bumper) {
            switchDir(-1);
        }

        driveTrain.teleOp(gamepad1);
        lift.teleOp(gamepad2);
        latches.teleOp(gamepad1);
        tape.teleOp(gamepad2);
    }

    public double getPathProgress() {
        return controller.getProgress();
    }


    public void followPath(VectorF[] points, double power) {
        controller = new PurePursuitController2D(this, new Path(points));
        controller.followPath(power);
    }

    public void followSplinedPath2D(VectorF[] points, double power) {
        controller = new PurePursuitController2D(this, new SplinePath(points));
        controller.followPath(power);
    }

    /*public void followPath3D(VectorF[] points, double power) {
        PurePursuitController3D controller = new PurePursuitController3D(this, new Path(points));
        controller.followPath(power);
    }*/

    public void followSplinedPath3D(VectorF[] points, double power) {
        controller = new PurePursuitController3D(this, new SplinePath(points));
        controller.followPath(power);
    }




    private void show(TelemetryPacket packet) {
        shape.show(packet, "green");
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
        for(VectorF v : breadCrumbs) {
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

    public void displayDash(Path path, VectorF lookAheadPoint, VectorF followVec,
                            VectorF toPathVec, VectorF repulsionVec, VectorF finalVec,
                            HashMap<String, Double> valuesToGraph) {
        if(dashboard == null) return;
        TelemetryPacket packet = new TelemetryPacket();

        field.show(packet, "red");

        if(path != null) {
            path.show(packet, "black");
        }

        showBreadCrumbs(packet, "brown");

        if(lookAheadPoint != null) {
            VectorF pos = position;

            packet.fieldOverlay().setStroke("orange").strokeLine(pos.get(0), pos.get(1),
                    lookAheadPoint.get(0), lookAheadPoint.get(1));
            packet.fieldOverlay().setStroke("orange").setFill("orange").fillCircle(
                    lookAheadPoint.get(0), lookAheadPoint.get(1), 1.5);
        }

        this.show(packet);

        drawArrow(packet, get2DPosition(), get2DPosition().added(followVec), "orange");
        drawArrow(packet, get2DPosition(), get2DPosition().added(toPathVec), "blue");
        drawArrow(packet, get2DPosition(), get2DPosition().added(repulsionVec), "red");

        drawArrow(packet, get2DPosition(), get2DPosition().added(finalVec.multiplied(12)), "black");

        if(breadCrumbTimer.milliseconds() > 200) {
            breadCrumbs.add(getPosition());
            breadCrumbTimer.reset();
        }

        sendValues(packet, valuesToGraph);

        dashboard.sendTelemetryPacket(packet);
    }

    public void displayDash(Path path, @NonNull VectorF lookAheadPoint, VectorF arcCenter,
                            double toPath, HashMap<String, Double> valuesToGraph) {
        if(dashboard == null) return;
        TelemetryPacket packet = new TelemetryPacket();

        field.show(packet, "red");

        if(path != null) {
            path.show(packet, "black");
        }

        showBreadCrumbs(packet, "brown");

        VectorF pos = position;

        //packet.fieldOverlay().setStroke("orange").strokeLine(pos.get(0), pos.get(1),
        //        lookAheadPoint.get(0), lookAheadPoint.get(1));
        packet.fieldOverlay().setStroke("orange").setFill("orange").fillCircle(
                lookAheadPoint.get(0), lookAheadPoint.get(1), 1.5);


        this.show(packet);

        //Utilities.drawArrow(packet, get2DPosition(), get2DPosition().added(toPathVec), "blue");
        drawLocalArrow(packet, new VectorF(0, 0), new VectorF(-(float)toPath*12, 0),
                getPosition(), "blue");
        if(arcCenter == null) {
            packet.fieldOverlay().setStroke("orange").strokeLine(get2DPosition().get(0),
                    get2DPosition().get(1), lookAheadPoint.get(0), lookAheadPoint.get(1));
        } else {
            //Utilities.drawArc(packet, get2DPosition(), lookAheadPoint, arcCenter, 10, "orange");
            packet.fieldOverlay().setStroke("orange").strokeCircle(arcCenter.get(0),
                    arcCenter.get(1), distance(arcCenter, lookAheadPoint));
            packet.fieldOverlay().setStroke("orange").strokeCircle(arcCenter.get(0),
                    arcCenter.get(1), 2);
        }


        if(breadCrumbTimer.milliseconds() > 200) {
            breadCrumbs.add(getPosition());
            breadCrumbTimer.reset();
        }

        sendValues(packet, valuesToGraph);

        dashboard.sendTelemetryPacket(packet);
    }


    public void displayDash(Path path, VectorF lookAheadPoint) {
        if(dashboard == null) return;
        TelemetryPacket packet = new TelemetryPacket();

        if(path != null) {
            path.show(packet, "black");
        }

        showBreadCrumbs(packet, "brown");

        if(lookAheadPoint != null) {
            VectorF pos = position;

            packet.fieldOverlay().setStroke("orange").strokeLine(pos.get(0), pos.get(1),
                    lookAheadPoint.get(0), lookAheadPoint.get(1));
            packet.fieldOverlay().setStroke("orange").setFill("orange").fillCircle(
                    lookAheadPoint.get(0), lookAheadPoint.get(1), 1.5);
        }

        this.show(packet);

        if(breadCrumbTimer.milliseconds() > 200) {
            breadCrumbs.add(getPosition());
            breadCrumbTimer.reset();
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void displayDash() {
        if(dashboard == null) return;
        TelemetryPacket packet = new TelemetryPacket();

        field.show(packet, "red");

        this.show(packet);

        //breadCrumbs.add(getPosition());

        dashboard.sendTelemetryPacket(packet);
    }

}
