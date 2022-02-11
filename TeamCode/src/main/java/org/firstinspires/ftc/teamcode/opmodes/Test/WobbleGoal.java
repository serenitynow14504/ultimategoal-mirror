package org.firstinspires.ftc.teamcode.opmodes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.VectorD;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Robot;

@Autonomous
@Disabled
public class WobbleGoal extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD,
                72-RobotConstants.width, 0, 0);

        robot.init(hardwareMap, Robot.VisionMethod.TENSORFLOW);

        while(!isStopRequested() && !opModeIsActive()) {
            robot.detectStack();
            telemetry.addData("pos", robot.getBarcodeState());
            telemetry.addData("time", getRuntime());
            telemetry.update();
            sleep(200);
        }

        waitForStart();
        robot.start();

        VectorD target = FieldConstants.getTargetZone(robot.getBarcodeState());


        sleep(1000);

        robot.followPath(0.6,
                new VectorD(76, 40), target);


        //robot.driveTrain.setScaledPowersFromGlobalVector(new VectorD(0, 0.3), 0);

        //robot.wobbleArm.grabber(false);


    }
}
