package org.firstinspires.ftc.teamcode.OpModes.Old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@TeleOp
public class odoTest extends LinearOpMode {
    private Robot robot = new Robot(this, RobotConstants.ALLIANCES.RED, FieldConstants.SKYSTONE_FIELD, -1, -1, 0, 0, 0);


    @Override
    public void runOpMode() {
        robot.INIT();

        waitForStart();

        robot.begin();

        while(opModeIsActive()) {
            VectorF pos = robot.getPosition();
            telemetry.addData("position:  ", pos.toString());
            //telemetry.addData("encX pos = ", robot.odometry.getEncPos(0));
            //telemetry.addData("encY pos = ", robot.odometry.getEncPos(1));
            telemetry.update();
        }
    }
}