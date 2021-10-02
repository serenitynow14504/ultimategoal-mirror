package org.firstinspires.ftc.teamcode.OpModes.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@TeleOp
@Disabled
public class TeleOpRaw extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime speedTimer = new ElapsedTime();

    private DcMotor tapeWorm;



    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        tapeWorm = hardwareMap.get(DcMotor.class, "encX");
        tapeWorm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();
        speedTimer.reset();



        while (opModeIsActive()) {
            tapeWorm.setPower(gamepad1.right_stick_y);

            telemetry.update();
        }
    }
}
