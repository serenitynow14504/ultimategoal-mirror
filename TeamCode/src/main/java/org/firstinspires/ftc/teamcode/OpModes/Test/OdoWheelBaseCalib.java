package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Util;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Imu;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;
import org.firstinspires.ftc.teamcode.RobotComponents.ThreeWheelOdometry;

@TeleOp
public class OdoWheelBaseCalib extends LinearOpMode {
    Robot robot;
//    File FLWheelSpeedsFile = AppUtil.getInstance().getSettingsFile("FLWheelSpeeds.txt");
//    File BLWheelSpeedsFile = AppUtil.getInstance().getSettingsFile("BLWheelSpeeds.txt");
//    File FRWheelSpeedsFile = AppUtil.getInstance().getSettingsFile("FRWheelSpeeds.txt");
//    File BRWheelSpeedsFile = AppUtil.getInstance().getSettingsFile("BRWheelSpeeds.txt");


    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD, 0,
                0, 0, true);
        robot.init(hardwareMap, false);
        Imu imu = new Imu(hardwareMap.get(BNO055IMU.class, "imu"), robot);
        imu.setAngles();
        Util.log("initafjdhaksd");

        waitForStart();
        //robot.begin();

        imu.getData();
        Util.log("imu reset");
        ((ThreeWheelOdometry)robot.odometry).FL.getData();
        ((ThreeWheelOdometry)robot.odometry).FR.getData();
        Util.log("enc reset");
        robot.driveTrain.powers.setTankStrafe(0.4, -0.4, 0);
        robot.driveTrain.setPowers();
        sleep(1000);
        robot.driveTrain.powers.set(0);
        robot.driveTrain.setPowers();

        int leftTicks = ((ThreeWheelOdometry)robot.odometry).FL.getData()[1];
        int rightTicks = ((ThreeWheelOdometry)robot.odometry).FR.getData()[1];
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double angle = 0;
        int count = 0;
        while(timer.milliseconds()<500) {
            angle += imu.getData(true)[1];
            count++;
        }
        angle /= count;

        double difference = Math.abs(leftTicks) + Math.abs(rightTicks);
        double offsetPerDegree = difference/angle;

        double distance = (2*90*offsetPerDegree)/(Math.PI*RobotConstants.INCHES_TO_COUNTS_CONVERSION);
        Util.log(distance);

    }
}
