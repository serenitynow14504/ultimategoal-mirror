package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.HashMap;

@TeleOp(name = "_TeleOp", group = "_MAIN")
public class TeleOp_Ultimate_Goal extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD);
        robot.init(hardwareMap, true);

        waitForStart();

        robot.begin();

        while(opModeIsActive()) {
            robot.teleOp(gamepad1, gamepad2);

            telemetry.addData("Position: ",
                    "(" + robot.getPose().getX() + ", " + robot.getPose().getY() + ")");
            VectorD vP;
            String vPdisp = "";

            vP = null;//robot.getVuforiaPose();
            if(vP != null) {
                vPdisp = "Vuforia Position: (" + vP.getX() + ", " + vP.getY() + ", " + vP.getZ() + ")";
            } else {
                vPdisp = "No Vuforia targets visible";
            }
            robot.displayDash(new String[] {vPdisp, "Robot Pose: "+robot.getPose().toString(),
                    (robot.lift.usingEncoder ? "Using encoder" : "Using Distance Sensor")},
                    new HashMap<String, Double>() {{
                        put("intake draw", robot.intake.intakeWheels.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
                        put("lift pos", robot.lift.getLiftHeightMM());
                        put("flywheel vel", robot.shooter.getVel());
                        put("battery voltage", robot.getBatteryVoltage());
                    }});
            telemetry.update();
        }
    }
}
