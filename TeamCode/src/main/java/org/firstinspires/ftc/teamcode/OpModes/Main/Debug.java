package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@TeleOp
public class Debug extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD, 0, 0, 0);
        robot.INIT(hardwareMap, true);

        waitForStart();

        robot.begin();

        while(opModeIsActive()) {
            robot.debugOp(gamepad1, gamepad2);

            telemetry.addData("Position: ",
                    "(" + robot.getPose().getX() + ", " + robot.getPose().getY() + ")");
            VectorD vP;
            String vPdisp = "";
            try {
                vP = robot.getVuforiaPose();
                vPdisp = "Vuforia Position: (" + vP.getX() + ", " + vP.getY() + ", "
                        + vP.getZ() + ")";
            } catch (Exception e) {
                vPdisp = "No Vuforia targets visible";
            }
            robot.displayDash(new String[] {vPdisp});
            telemetry.update();
        }
    }
}
