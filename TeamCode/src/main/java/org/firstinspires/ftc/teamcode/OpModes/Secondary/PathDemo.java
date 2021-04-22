package org.firstinspires.ftc.teamcode.OpModes.Secondary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@Autonomous
@Disabled
public class PathDemo extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.DEMO, 0,
                0, 0, true);
        
        robot.init(hardwareMap, false);

        robot.detectStack();

        waitForStart();
        robot.begin();

        VectorD[] path = new VectorD[]{
                new VectorD(0, 0),
                new VectorD(-24, 24),
                new VectorD(24, 24),
                new VectorD(0, 48)
        };
        robot.followSplinedPath2D(path, 0.6);
    }
}
