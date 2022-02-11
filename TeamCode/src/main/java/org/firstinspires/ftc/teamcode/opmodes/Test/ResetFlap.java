package org.firstinspires.ftc.teamcode.opmodes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Robot;

@TeleOp(name="ResetFlap", group="Debug")
@Disabled
public class ResetFlap extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD, 0,
                0, 0,
                true);

        robot.init(hardwareMap, Robot.VisionMethod.NONE);
        //robot.shooter.flap(0);
        waitForStart();


    }
}
