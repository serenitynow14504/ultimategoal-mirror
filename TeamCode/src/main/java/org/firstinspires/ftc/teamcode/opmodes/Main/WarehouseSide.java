package org.firstinspires.ftc.teamcode.opmodes.Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.VectorD;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Robot;

@Autonomous
public class WarehouseSide extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.RED, FieldConstants.EMPTY_FIELD, 84,
                RobotConstants.width/2, -90, true);

        robot.init(hardwareMap, Robot.VisionMethod.OPENCV);




        //waitForStart 2.0
        while(!isStopRequested() && !opModeIsActive()) {
            robot.detectBarcode();
        }
        robot.start();

        robot.carouselWheel.on();

        robot.lift.setLevel(robot.getBarcodeState());

        //potential wait here

        robot.followPath(0.5, new VectorD(66, 18, -135), new VectorD(60, 30, -180));

        robot.lift.deposit();


        robot.followPath(0.5, new VectorD(66, 18, -135),
                new VectorD(80, RobotConstants.width/2, -90),
                new VectorD(112, RobotConstants.width/2, -90),
                new VectorD(112, 32, -90));

    }
}
