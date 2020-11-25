package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@Autonomous
public class WobbleGoal extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD, 1, 1,
                72-RobotConstants.width, 0, 0);

        robot.INIT(hardwareMap);

        while(!isStopRequested() && !opModeIsActive()) {
            robot.detectStack(telemetry);
            telemetry.addData("pos", robot.getStackState());
            telemetry.addData("time", getRuntime());
            telemetry.update();
            sleep(200);
        }

        waitForStart();
        robot.begin();

        VectorD target = FieldConstants.getTargetZone(robot.getStackState());


        VectorD[] path = new VectorD[] {
                robot.getPosition(),
                new VectorD(76, 40),
                target
        };

        telemetry.addData("path length", path.length);
        telemetry.update();
        sleep(1000);

        robot.followPath2D(path, 0.6);
        //robot.driveTrain.setScaledPowersFromGlobalVector(new VectorD(0, 0.3), 0);

        //robot.wobbleArm.grabber(false);


    }
}
