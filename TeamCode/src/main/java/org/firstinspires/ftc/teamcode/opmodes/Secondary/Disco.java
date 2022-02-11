package org.firstinspires.ftc.teamcode.opmodes.Secondary;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Robot;

@TeleOp
public class Disco extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD, 0,
                0, 0,
                true);

        robot.init(hardwareMap, Robot.VisionMethod.NONE);

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
