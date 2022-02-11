package org.firstinspires.ftc.teamcode.opmodes.Test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Util;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Imu;
import org.firstinspires.ftc.teamcode.robotComponents.Robot;
import org.firstinspires.ftc.teamcode.robotComponents.ThreeWheelOdometry;

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
        robot.init(hardwareMap, Robot.VisionMethod.NONE);
        Imu imu = new Imu(hardwareMap.get(BNO055IMU.class, "imu"), robot);
        imu.setAngles();
        Util.log("initafjdhaksd");

        waitForStart();
        //robot.start();

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
