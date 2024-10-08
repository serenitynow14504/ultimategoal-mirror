package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.Util;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@TeleOp(name="ResetPoseFile", group="Debug")
public class ResetPoseFile extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD, 0,
                0, 0, true);
        try {
            Util.log(robot.readPose().toString());
        } catch (Exception e) {
            Util.log("no pose to read");}
        robot.writePose();

        waitForStart();
    }
}
