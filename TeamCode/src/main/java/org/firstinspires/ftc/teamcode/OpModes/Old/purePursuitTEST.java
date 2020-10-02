package org.firstinspires.ftc.teamcode.OpModes.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Odometry;
import org.firstinspires.ftc.teamcode.RobotComponents.PurePursuitDrivetrain;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@Autonomous
public class purePursuitTEST extends LinearOpMode {
    Robot r = new Robot(this, RobotConstants.ALLIANCES.RED, FieldConstants.SKYSTONE_FIELD, 1, 1, 0, 0, 0, true);

    @Override
    public void runOpMode() {

        r.odometry.setMode(Odometry.MODES.LINE);
        r.setDriveTrainPurePursuit();
        PurePursuitDrivetrain purePursuitDrivetrain = (PurePursuitDrivetrain)r.driveTrain;

        r.INIT();


        waitForStart();

        r.begin();

        //sleep(1000);

        VectorF[] points = new VectorF[] {
            new VectorF(0, 0),
                new VectorF(0, -10),
                new VectorF(96, -14),
                new VectorF(96, -4)
        };

        purePursuitDrivetrain.setPath(points);


        purePursuitDrivetrain.followPathNoPID(0.4);
    }
}
