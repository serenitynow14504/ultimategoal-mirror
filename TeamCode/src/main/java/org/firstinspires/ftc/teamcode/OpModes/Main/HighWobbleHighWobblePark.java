package org.firstinspires.ftc.teamcode.OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.MovementControllers.PurePursuitController2D;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

@Autonomous(name="HighWobbleHighWobblePark", group="_MAIN")
public class HighWobbleHighWobblePark extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, RobotConstants.ALLIANCES.SOLO, FieldConstants.EMPTY_FIELD,
                48, 0, 0);

        robot.init(hardwareMap, false);

        robot.wobbleArm.grabber(true);
        sleep(500);
        robot.wobbleArm.arm(false);

        while(!isStopRequested() && !opModeIsActive()) {
            robot.detectStack();
            telemetry.addData("pos", robot.getStackState());
            telemetry.addData("time", getRuntime());
            telemetry.update();
            sleep(200);
        }
        robot.setAimPos(FieldConstants.HIGH_GOAL);

        waitForStart();
        robot.begin();

        int stackState = robot.getStackState();

        VectorD target = FieldConstants.getTargetZone(stackState);
        VectorD targetZonePose = new VectorD(target.getX()-8, target.getY() - 15, -90);


        VectorD[] path = new VectorD[] {
                robot.getPose(),
                new VectorD(36, 40, 0),
                new VectorD(36, 60, -1)
        };

        robot.shooter.set(true);

        sleep(500);
        robot.aim();
        robot.followPath2D(path, 0.6);

        sleep(500);

        robot.shooter.push();
        //sleep(500);
        robot.shooter.push();
        //sleep(500);
        robot.shooter.push();

        robot.shooter.set(false);
        robot.stopAim();

        /*path = new VectorD[] {
                robot.getPose(),
                targetZonePose.added(new VectorD(0, 5, 0))
        };

        robot.followPath2D(path, 0.6);*/
        robot.posOnRobotToGlobalPos(RobotConstants.GRABBER_POS, target, (stackState==2)?0.85 : 0.7,
                -2, 5000);
        sleep(250);


        //robot.shooter.push();

        robot.wobbleArm.release();

        sleep(150);

        robot.wobbleArm.arm(true);
        if(stackState > 0) {
            robot.lift.liftDown();
            robot.intake.on();
            //robot.shooter.pusherWall();
            if(stackState==1) {
                robot.posOnRobotToGlobalPos(RobotConstants.INTAKE_POS, FieldConstants.RING_STACK,
                        0.7, 5);
            } else {
                robot.posOnRobotToGlobalPos(RobotConstants.INTAKE_POS, FieldConstants.RING_STACK,
                        0.7, -14);
                //sleep(100);
                robot.posOnRobotToGlobalPos(RobotConstants.INTAKE_POS, FieldConstants.RING_STACK,
                        0.15, 0);
                /*sleep(1000);
                while(!robot.intake.doneIntaking()) {
                    sleep(50);
                }
                robot.intake.off();

                robot.shooter.set(true);
                robot.lift.liftUp();

                robot.aim();

                path = new VectorD[] {
                        robot.getPose(),
                        new VectorD(60, 60, -1)
                };

                sleep(200);

                robot.followPath2D(path, 0.6);

                robot.shooter.push();
                sleep(500);
                robot.shooter.push();
                sleep(500);
                robot.shooter.push();
                robot.stopAim();

                robot.shooter.set(false);
                robot.lift.liftDown();
                robot.intake.on();
                robot.posOnRobotToGlobalPos(RobotConstants.INTAKE_POS, new VectorD(54, 30), 0.3
                        , 5);*/
            }
            //sleep(500);
            while(!robot.intake.doneIntaking()) {
                sleep(50);
            }

            sleep(1000);

            robot.wobbleArm.arm(true);
            robot.wobbleArm.grabber(false);
            robot.posOnRobotToGlobalPos(RobotConstants.GRABBER_POS, FieldConstants.RIGHT_WOBBLE,
                    0.3, -14);
            PurePursuitController2D.TpK_P = 1.5;
            PurePursuitController2D.TpK_D = 1.75;
            robot.posOnRobotToGlobalPos(RobotConstants.GRABBER_POS, FieldConstants.RIGHT_WOBBLE,
                    0.45, 2, 3000);
            PurePursuitController2D.TpK_P = 2.2;
            PurePursuitController2D.TpK_D = 2.2;
            robot.wobbleArm.grabber(true);
            sleep(400);
            robot.wobbleArm.arm(false);

            robot.intake.off();
            robot.shooter.set(true);
            robot.lift.liftUp();

            robot.aim();
            path = new VectorD[] {
                    robot.getPose(),
                    new VectorD(60, 60, -1)
            };

            sleep(200);

            robot.followPath2D(path, 0.6);

            while(!robot.driveTrain.heading.onTarget()) {
                sleep(50);
            }

            robot.shooter.push();
            if(stackState == 2) {
                robot.shooter.push();
                robot.shooter.push();
            }
            robot.stopAim();

            robot.shooter.set(false);

        } else {
            /*robot.wobbleArm.arm(true);
            robot.wobbleArm.grabber(false);
            robot.posOnRobotToGlobalPos(RobotConstants.GRABBER_POS, FieldConstants.RIGHT_WOBBLE,
                    0.4, 0);
            robot.wobbleArm.grabber(true);
            sleep(400);
            robot.wobbleArm.arm(false);*/
            robot.wobbleArm.arm(true);
            robot.wobbleArm.grabber(false);
            robot.posOnRobotToGlobalPos(RobotConstants.GRABBER_POS, FieldConstants.RIGHT_WOBBLE,
                    0.4, -14);
            PurePursuitController2D.TpK_P = 1.35;
            PurePursuitController2D.TpK_D = 1.6;
            robot.posOnRobotToGlobalPos(RobotConstants.GRABBER_POS, FieldConstants.RIGHT_WOBBLE,
                    0.45, -2.5);
            PurePursuitController2D.TpK_P = 2.2;
            PurePursuitController2D.TpK_D = 2.2;
            robot.wobbleArm.grabber(true);
            robot.shooter.pushers(false);
            sleep(400);
            robot.wobbleArm.arm(false);
        }

        /*path = new VectorD[] {
                robot.getPose(),
                targetZonePos
        };

        robot.followPath2D(path, 0.9);*/
        robot.posOnRobotToGlobalPos(RobotConstants.GRABBER_POS, target, 0.75, -6);
        sleep(150);
        robot.wobbleArm.release();

        path = new VectorD[] {
                robot.getPose(),
                new VectorD(54, 84, -90)
        };

        robot.followPath2D(path, 0.9);
        robot.wobbleArm.arm(false);
    }
}
