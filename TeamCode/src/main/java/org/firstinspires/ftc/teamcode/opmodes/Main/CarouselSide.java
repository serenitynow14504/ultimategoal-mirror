package org.firstinspires.ftc.teamcode.opmodes.Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.VectorD;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Robot;

@Autonomous
public class CarouselSide extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.RED, FieldConstants.EMPTY_FIELD, 36,
                RobotConstants.width/2, 90, true);

        robot.init(hardwareMap, Robot.VisionMethod.OPENCV);




        //waitForStart 2.0
        while(!isStopRequested() && !opModeIsActive()) {
            robot.detectBarcode();
        }
        robot.start();

        robot.carouselWheel.on();

        robot.followPath(0.4, new VectorD(20, RobotConstants.width/2));
        sleep(1000);

        robot.lift.setLevel(robot.getBarcodeState());

        //potential wait here

        if(robot.getBarcodeState() == 2) {
            robot.followPath(0.5, new VectorD(24, 48), new VectorD(40, 48));
        } else {
            robot.followPath(0.5, new VectorD(40, 12, 145), new VectorD(48, 27, 145));
        }

        robot.lift.deposit();

        if(robot.getBarcodeState() == 2) {
            robot.followPath(0.5, new VectorD(30, 48, 180),
                    new VectorD(30, 20, 210),
                    new VectorD(72, RobotConstants.width/2, 270),
                    new VectorD(112, RobotConstants.width/2, 270));
        } else {
            robot.followPath(0.5, new VectorD(48, 24, 210),
                    new VectorD(72, RobotConstants.width/2, 270),
                    new VectorD(112, RobotConstants.width/2, 270));
        }


    }
}
