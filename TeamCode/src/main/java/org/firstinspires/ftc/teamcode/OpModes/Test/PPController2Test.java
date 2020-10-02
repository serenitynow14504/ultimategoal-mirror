package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Odometry;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@Autonomous
public class PPController2Test extends LinearOpMode {
    Robot r;

    @Override
    public void runOpMode() {
        r = new Robot(this, RobotConstants.ALLIANCES.RED, FieldConstants.EMPTY_FIELD, 1, 1, 0, 0, 0,
                true);
        r.odometry.setMode(Odometry.MODES.LINE);
        r.INIT();

        waitForStart();

        r.begin();

        //while(opModeIsActive()) {r.displayDash(null, null);}

        r.followSplinedPath3D(
            new VectorF[] {
                new VectorF(0, 0),
                new VectorF(0, 36),
                new VectorF(36, 36),
                new VectorF(36, 0),
                new VectorF(6, 0)
            }, 0.35);
    }
}
