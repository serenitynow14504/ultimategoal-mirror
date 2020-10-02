package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Odometry;
import org.firstinspires.ftc.teamcode.RobotComponents.PurePursuitDrivetrain;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;
import org.firstinspires.ftc.teamcode.RobotComponents.CV.Pipelines.skyStoneCV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class PPStoneAutoNoPullRED extends LinearOpMode {
    OpenCvCamera phoneCam;
    Robot robot = new Robot(this, RobotConstants.ALLIANCES.RED, FieldConstants.EMPTY_FIELD, 1, 1, 24, 0, 0);

    skyStoneCV pipeline = new skyStoneCV(this, robot);

    private int cycles = 3;

    @Override
    public void runOpMode() {

        robot.odometry.setMode(Odometry.MODES.LINE);
        robot.setDriveTrainPurePursuit();
        PurePursuitDrivetrain purePursuitDrivetrain = (PurePursuitDrivetrain)robot.driveTrain;

        robot.INIT();



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        phoneCam.setPipeline(pipeline);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        phoneCam.showFpsMeterOnViewport(false);

        waitForStart();

        robot.lift.grabber(false);

        robot.setSkyStoneOrder(pipeline.getFinalPos());
        phoneCam.stopStreaming();

        robot.begin();

        int stoneCounter = 0;


        robot.setTaskState(Robot.TaskState.GO_TO_STONE);
        VectorF[] points = new VectorF[] {
                robot.getPosition(),
                new VectorF((float)robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)), 40-(float)robot.getLength()),
                new VectorF((float)robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)), 50-(float)robot.getLength())
        };

        purePursuitDrivetrain.setPath(points);
        purePursuitDrivetrain.followPathNoPID(0.7);



        for(int i = 0; i < cycles - 1; i++) {
            robot.lift.grabber(true);

            //GOING TO FOUNDATION
            robot.setTaskState(Robot.TaskState.GO_TO_FOUNDATION);
            points = new VectorF[] {
                    robot.getPosition(),
                    new VectorF((float)robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)), 23, 0),
                    new VectorF(72, 32, -90),
                    new VectorF(120,28, -90),
            };

            purePursuitDrivetrain.setPath(points);
            purePursuitDrivetrain.followPathNoPID(0.8);
            stoneCounter++;

            robot.lift.grabber(false);
            robot.lift.increaseLiftHeight(400);

            //GOING BACK FOR A STONE
            robot.setTaskState(Robot.TaskState.GO_BACK_TO_STONE);
            points = new VectorF[]{
                    robot.getPosition(),
                    new VectorF(72, 32, 0),
                    new VectorF((float) robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)) + 9, 32),
                    new VectorF((float) robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter))+4, 54 - (float) robot.getLength())
            };
            purePursuitDrivetrain.setPath(points);
            purePursuitDrivetrain.followPathNoPID(0.8);

        }


        robot.lift.grabber(true);
        //DEPOSIT STONE AND GRAB FOUNDATION

        robot.setTaskState(Robot.TaskState.GO_TO_FOUNDATION);

        points = new VectorF[] {
                robot.getPosition(),
                new VectorF((float)robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)), 23, 0),
                new VectorF(72, 32, -90),
                new VectorF(120,28, -90),
        };

        purePursuitDrivetrain.setPath(points);
        purePursuitDrivetrain.followPathNoPID(0.8);

        robot.lift.grabber(false);

        robot.setTaskState((Robot.TaskState.IDLE));

        points = new VectorF[] {
                robot.getPosition(),
                new VectorF(72, 34)
        };

        purePursuitDrivetrain.setPath(points);
        purePursuitDrivetrain.followPathNoPID(0.55);



    }
}
