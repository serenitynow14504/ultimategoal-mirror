package org.firstinspires.ftc.teamcode.opmodes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.VectorD;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.robotComponents.Robot;

@Autonomous
@Disabled
public class PPControllerTest extends LinearOpMode {
    Robot r;

    @Override
    public void runOpMode() {
        r = new Robot(this, RobotConstants.ALLIANCES.RED, FieldConstants.TEST, 0, 0, 0,
                true);
        r.init(hardwareMap, Robot.VisionMethod.NONE);

        waitForStart();

        r.start();

        //while(opModeIsActive()) {r.displayDash(null, null);}

        r.followPath(0.5,
                new VectorD(0, 0),
                new VectorD(0, 36),
                new VectorD(36, 36),
                new VectorD(36, 0),
                new VectorD(6, 0));
    }
}
