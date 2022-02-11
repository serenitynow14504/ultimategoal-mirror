package org.firstinspires.ftc.teamcode.opmodes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "_TeleOp", group = "_MAIN")
public class TeleOp_FreightFrenzy extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD);
        robot.init(hardwareMap, Robot.VisionMethod.NONE);

        waitForStart();

        robot.start();

        while(opModeIsActive()) {
            robot.teleOp(gamepad1, gamepad2);

            telemetry.addData("Position: ",
                    "(" + robot.getPose().getX() + ", " + robot.getPose().getY() + ")");



            telemetry.update();
        }
    }
}
