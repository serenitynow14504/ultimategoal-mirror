package org.firstinspires.ftc.teamcode.OpModes.Secondary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@Autonomous
public class parktape extends LinearOpMode {
    Robot robot = new Robot(this, RobotConstants.ALLIANCES.BLUE, FieldConstants.SKYSTONE_FIELD, 1, 1, 24, 0, 0);



    @Override
    public void runOpMode() {


        robot.INIT();

        waitForStart();

        robot.tape.setPower(-1);
        sleep(500);
        robot.tape.setPower(0);
    }
}
