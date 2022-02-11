package org.firstinspires.ftc.teamcode.opmodes.Secondary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.VectorD;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Robot;

@Autonomous
//@Disabled
public class PathDemo extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.DEMO, 0,
                0, 0, true);
        
        robot.init(hardwareMap, Robot.VisionMethod.NONE);

        //robot.detectStack();//?????

        waitForStart();
        robot.start();

        VectorD[] path = new VectorD[]{
                new VectorD(0, 0),
                new VectorD(-24, 24),
                new VectorD(24, 24),
                new VectorD(0, 48)
        };
        robot.followPath(0.4, new VectorD(36, 0, 180));
    }
}
