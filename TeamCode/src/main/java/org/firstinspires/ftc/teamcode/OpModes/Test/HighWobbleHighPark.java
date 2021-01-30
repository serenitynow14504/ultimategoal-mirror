package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@Autonomous
public class HighWobbleHighPark extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD,
                72-RobotConstants.width, 0, 0);

        robot.INIT(hardwareMap, false);

        robot.wobbleArm.grabber(true);

        while(!isStopRequested() && !opModeIsActive()) {
            robot.detectStack();
            telemetry.addData("pos", robot.getStackState());
            telemetry.addData("time", getRuntime());
            telemetry.update();
            sleep(200);
        }

        waitForStart();
        robot.begin();

        int stackState = robot.getStackState();

        VectorD target = FieldConstants.getTargetZone(stackState);
        VectorD targetPose = new VectorD(target.getX(), target.getY() - 18, -90);


        VectorD[] path = new VectorD[] {
                robot.getPose(),
                new VectorD(36, 40, 0),
                new VectorD(36, 60, -1)
        };

        robot.shooter.setPower(-1);

        sleep(500);

        robot.followPath2D(path, 0.6);

        robot.shooter.push();
        sleep(500);
        robot.shooter.push();
        sleep(500);
        robot.shooter.push();

        robot.shooter.setPower(0);

        path = new VectorD[] {
                robot.getPose(),
                targetPose
        };

        robot.followPath2D(path, 0.6);

        robot.lift.liftDown();

        robot.wobbleArm.grabber(false);

        sleep(500);

        if(stackState == 1) {
            robot.intake.on();
            robot.posOnRobotToGlobalPos(RobotConstants.INTAKE_POS, FieldConstants.RING_STACK);
            sleep(1000);
            robot.intake.off();
            robot.lift.liftUp();

            path = new VectorD[] {
                    robot.getPose(),
                    new VectorD(36, 60, -1)
            };

            sleep(200);

            robot.followPath2D(path, 0.6);

            robot.shooter.push();

            robot.shooter.setPower(0);
        }

        path = new VectorD[] {
                robot.getPose(),
                new VectorD(60, 84, -90)
        };

        robot.followPath2D(path, 0.6);
    }
}
