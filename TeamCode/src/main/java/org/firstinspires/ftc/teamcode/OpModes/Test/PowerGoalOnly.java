package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@TeleOp
public class PowerGoalOnly extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD, 1, 1, 0,
                0, 0,
                true);


        waitForStart();

        while(opModeIsActive()) {
            robot.setLedColors(255, 0, 0);
            sleep(333);
            robot.setLedColors(0, 255, 0);
            sleep(333);
            robot.setLedColors(0, 0, 255);
            sleep(333);
        }

    }
}
