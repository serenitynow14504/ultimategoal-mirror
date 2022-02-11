package org.firstinspires.ftc.teamcode.opmodes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Nothing extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        while(opModeIsActive()) sleep(100);

    }
}
