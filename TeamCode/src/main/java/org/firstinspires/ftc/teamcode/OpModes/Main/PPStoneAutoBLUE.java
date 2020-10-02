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
public class PPStoneAutoBLUE extends LinearOpMode {
    OpenCvCamera phoneCam;
    Robot robot = new Robot(this, RobotConstants.ALLIANCES.BLUE, FieldConstants.EMPTY_FIELD, 1, 1, 24, 0,0);

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
                new VectorF((float)robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)), (float)50.5-(float)robot.getLength())
        };

        purePursuitDrivetrain.setPath(points);
        purePursuitDrivetrain.followPathNoPID(0.7);



        for(int i = 0; i < cycles - 1; i++) {
            robot.lift.grabber(true);

            //GOING TO FOUNDATION
            robot.setTaskState(Robot.TaskState.GO_TO_FOUNDATION);
            points = new VectorF[] {
                    robot.getPosition(),
                    new VectorF((float)robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)), 23),
                    new VectorF(72, (float)33),
                    new VectorF(120,(float)33),
                    new VectorF(120, (float)(48.5 + 2.5 * i -robot.getLength()/2))
            };

            purePursuitDrivetrain.setPath(points);
            purePursuitDrivetrain.followPathNoPID(0.8);
            stoneCounter++;

            robot.lift.grabber(false);
            robot.lift.increaseLiftHeight(320);
            //sleep(200);

            //GOING BACK FOR A STONE
            robot.setTaskState(Robot.TaskState.GO_BACK_TO_STONE);

            //robot.lift.grabber(true);

            points = new VectorF[]{
                    robot.getPosition(),
                    new VectorF(120, 36),
                    new VectorF(72, (float)33),
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
                new VectorF((float)robot.calculateSStonePos(robot.getSkyStoneInOrder(stoneCounter)), 26),
                new VectorF(72, 32 ),
                new VectorF(120,32),
                new VectorF(120, (float)(44-robot.getLength()/2))
        };

        purePursuitDrivetrain.setPath(points);
        purePursuitDrivetrain.followPathNoPID(0.8);

        robot.setTaskState((Robot.TaskState.GRAB_FOUNDATION));

        points = new VectorF[] {
                robot.getPosition(),
                new VectorF(120, 56-(float)robot.getLength()/2)
        };

        purePursuitDrivetrain.setPath(points);
        purePursuitDrivetrain.followPathNoPID(0.55);


        robot.lift.grabber(false);

        //MOVE FOUNDATION
        robot.setTaskState(Robot.TaskState.MOVE_FOUNDATION);

        points = new VectorF[] {
                robot.getPosition(),
                new VectorF(128, (float)robot.getLength()/2 - (float)1.5)
        };

        purePursuitDrivetrain.setPath(points);
        purePursuitDrivetrain.followPathNoPID(0.8);


        robot.latches.latch(false);


        //PARKING
        robot.setTaskState(Robot.TaskState.PARK);

        points = new VectorF[] {
                robot.getPosition(),
                new VectorF(100, (float)robot.getLength()/2, 0),
                new VectorF(94, (float)robot.getLength()/2,45)
        };

        purePursuitDrivetrain.setPath(points);
        purePursuitDrivetrain.followPathAbsolutelyNoPID(0.6, 0.4);

        //purePursuitDrivetrain.rotate(-55, 0.65, 4000);
        //move forward later
        while(opModeIsActive()) {}
    }
}
