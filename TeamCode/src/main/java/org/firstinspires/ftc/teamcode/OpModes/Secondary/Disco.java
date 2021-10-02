package org.firstinspires.ftc.teamcode.OpModes.Secondary;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@TeleOp
public class Disco extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD, 0,
                0, 0,
                true);

        robot.init(hardwareMap, false);

        waitForStart();

        while(opModeIsActive()) {
            robot.setLedColors(255, 0, 0);
            sleep(300);
            robot.setLedColors(0, 255, 0);
            sleep(300);
            robot.setLedColors(0, 0, 255);
            sleep(300);
        }


    }
}
