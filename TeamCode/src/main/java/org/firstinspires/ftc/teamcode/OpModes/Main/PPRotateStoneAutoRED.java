package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
public class PPRotateStoneAutoRED extends LinearOpMode {
    OpenCvCamera phoneCam;
    Robot robot = new Robot(this, RobotConstants.ALLIANCES.RED, FieldConstants.EMPTY_FIELD, 1, 1, 24, 0, 0);

    skyStoneCV pipeline = new skyStoneCV(this, robot);

    private final int cycles = 2;

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
                new VectorF((float)robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)), 40-(float)robot.getLength(), 0),
                new VectorF((float)robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)), 54-(float)robot.getLength(), 0)
        };

        purePursuitDrivetrain.setPath(points);
        purePursuitDrivetrain.followPathNoPID(0.5);



        for(int i = 0; i < cycles; i++) {
            robot.lift.grabber(true);

            //GOING TO FOUNDATION
            robot.setTaskState(Robot.TaskState.GO_TO_FOUNDATION);
            points = new VectorF[] {
                    robot.getPosition(),
                    new VectorF((float)robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)), 26, 0),
                    new VectorF(48, 32, -90),
                    new VectorF(72, 32, 0),
                    new VectorF(86, 30, 0),
                    new VectorF(120,32, 0),
                    new VectorF(120, (float)(52-robot.getLength()/2), 0)
            };

            purePursuitDrivetrain.setPath(points);
            purePursuitDrivetrain.followPathNoPID(0.65);
            stoneCounter++;

            if(i == cycles - 1) {break;}

            robot.lift.grabber(false);

            //GOING BACK FOR A STONE
            robot.setTaskState(Robot.TaskState.GO_BACK_TO_STONE);
            points = new VectorF[]{
                    robot.getPosition(),
                    new VectorF(120, 36, 0),
                    new VectorF(105, 34, 90),
                    new VectorF(72, 32, 90),
                    new VectorF(60, 30, 0),
                    new VectorF((float) robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)) + 9, 32, 0),
                    new VectorF((float) robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)), 54 - (float) robot.getLength(), 0)
            };
            purePursuitDrivetrain.setPath(points);
            purePursuitDrivetrain.followPathAbsolutelyNoPID(0.65, 0.4);

        }


        /*robot.lift.grabber(true);
        //DEPOSIT STONE AND GRAB FOUNDATION

        robot.setTaskState(Robot.TaskState.GO_TO_FOUNDATION);

        points = new VectorF[] {
                robot.getPosition(),
                new VectorF((float)robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)), 26, 0),
                new VectorF((float)robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)) + 20, 32, -90),
                new VectorF(72, 32, -90),
                new VectorF(100, 30, 0),
                new VectorF(120,32, 0),
                new VectorF(120, (float)(42-robot.getLength()/2), 0)
        };

        purePursuitDrivetrain.setPath(points);
        purePursuitDrivetrain.followPathAbsolutelyNoPID(0.65, 0.4);*/

        robot.setTaskState((Robot.TaskState.GRAB_FOUNDATION));

        points = new VectorF[] {
                robot.getPosition(),
                new VectorF(120, 56-(float)robot.getLength()/2, 0)
        };

        purePursuitDrivetrain.setPath(points);
        purePursuitDrivetrain.followPathNoPID(0.45);


        robot.lift.grabber(false);

        //MOVE FOUNDATION
        robot.setTaskState(Robot.TaskState.MOVE_FOUNDATION);

        points = new VectorF[] {
                robot.getPosition(),
                new VectorF(126, (float)robot.getLength()/2, 0)
        };

        purePursuitDrivetrain.setPath(points);
        purePursuitDrivetrain.followPathNoPID(0.7);


        robot.latches.latch(false);


        //PARKING
        robot.setTaskState(Robot.TaskState.PARK);

        points = new VectorF[] {
                robot.getPosition(),
                new VectorF(110, (float)robot.getLength()/2, 0),
                new VectorF(100, (float)robot.getLength()/2, 65)
        };

        purePursuitDrivetrain.setPath(points);
        purePursuitDrivetrain.followPathAbsolutelyNoPID(0.6, 0.4);

        //purePursuitDrivetrain.rotate(-55, 0.65, 4000);
        //move forward later
        while(opModeIsActive()) {}
    }
}
