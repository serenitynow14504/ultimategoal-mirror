package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Odometry;
import org.firstinspires.ftc.teamcode.RobotComponents.PurePursuitDrivetrain;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@TeleOp
public class TeleOp_Skystone extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.RED, FieldConstants.EMPTY_FIELD, 1, 1, 0, 0, 0,
                true);
        robot.odometry.setMode(Odometry.MODES.LINE);
        robot.setDriveTrainPurePursuit();
        PurePursuitDrivetrain purePursuitDrivetrain = (PurePursuitDrivetrain)robot.driveTrain;

        robot.INIT();



        waitForStart();

        robot.begin();

        while(opModeIsActive()) {
            robot.teleOp(gamepad1, gamepad2);
            //purePursuitDrivetrain.displayDash(robot.getPosition(), null, null);
            robot.displayDash();
        }
    }
}
