package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

import java.io.File;

@TeleOp
public class MotorSpeedCalib extends LinearOpMode {
    Robot robot;
    File FLWheelSpeedsFile = AppUtil.getInstance().getSettingsFile("FLWheelSpeeds.txt");
    File BLWheelSpeedsFile = AppUtil.getInstance().getSettingsFile("BLWheelSpeeds.txt");
    File FRWheelSpeedsFile = AppUtil.getInstance().getSettingsFile("FRWheelSpeeds.txt");
    File BRWheelSpeedsFile = AppUtil.getInstance().getSettingsFile("BRWheelSpeeds.txt");


    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD, 1, 1, 0,
                0, 0,
                true);
        telemetry.addLine("==============================================================");
        telemetry.addLine("MAKE SURE ROBOT IS PROPPED UP AND WHEELS ARE NOT ON THE GROUND");
        telemetry.addLine("==============================================================");
        telemetry.update();

        waitForStart();

        robot.driveTrain.setPowers(0.2);
        sleep(3000);

        ReadWriteFile.writeFile(FLWheelSpeedsFile, String.valueOf(1));
    }

    void logPower(double power) {
        robot.driveTrain.setPowers(0.2);
        sleep(3000);

    }
}
