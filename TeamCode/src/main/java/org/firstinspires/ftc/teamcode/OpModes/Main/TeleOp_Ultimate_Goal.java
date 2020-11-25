package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@TeleOp
public class TeleOp_Ultimate_Goal extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD, 1, 1, 0,
                0, 0,
                true);
        robot.INIT(hardwareMap);

        waitForStart();

        robot.begin();

        while(opModeIsActive()) {
            robot.teleOp(gamepad1);
            robot.displayDash();
            telemetry.addData("Position: ",
                    "(" + robot.getPosition().getX() + ", " + robot.getPosition().getY() + ")");
            telemetry.update();
        }
    }
}
