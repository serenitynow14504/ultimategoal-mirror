package org.firstinspires.ftc.teamcode.robotComponents;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselWheel extends Capability {
    private DcMotor wheel;

    public CarouselWheel(Robot robot) {
        super(robot);
    }

    @Override
    void teleOp(Gamepad gamepad1, Gamepad gamepad2) {

    }

    @Override
    void init(HardwareMap hardwareMap) {
        //wheel = hardwareMap.get(DcMotor.class, "wheel");
    }

    @Override
    void tick(TelemetryPacket packet) {

    }

    public void on() {

    }

    public void off() {

    }
}
