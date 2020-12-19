package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@Autonomous
public class PPController2Test extends LinearOpMode {
    Robot r;

    @Override
    public void runOpMode() {
        r = new Robot(this, RobotConstants.ALLIANCES.RED, FieldConstants.EMPTY_FIELD, 0, 0, 0,
                true);
        r.odometry.setMode(OdometryLocalizer.MODES.LINE);
        r.INIT(hardwareMap, false);

        waitForStart();

        r.begin();

        //while(opModeIsActive()) {r.displayDash(null, null);}

        r.followSplinedPath3D(
            new VectorD[] {
                new VectorD(0, 0),
                new VectorD(0, 36),
                new VectorD(36, 36),
                new VectorD(36, 0),
                new VectorD(6, 0)
            }, 0.35);
    }
}
