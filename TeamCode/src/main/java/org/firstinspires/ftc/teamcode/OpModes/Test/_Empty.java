package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@Disabled
public class _Empty extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD, 0,
                0, 0,
                true);


        waitForStart();


    }
}
