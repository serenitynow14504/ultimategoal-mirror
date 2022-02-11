package org.firstinspires.ftc.teamcode.opmodes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Robot;

@TeleOp
public class Debug extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD, 0, 0, 0);
        robot.init(hardwareMap, Robot.VisionMethod.VUFORIA);
        waitForStart();
        robot.start();

        while(opModeIsActive()) {
            robot.debugOp(gamepad1, gamepad2);

        }
    }
}
