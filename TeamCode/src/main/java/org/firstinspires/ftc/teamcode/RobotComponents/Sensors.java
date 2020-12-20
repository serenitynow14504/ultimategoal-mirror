package org.firstinspires.ftc.teamcode.RobotComponents;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sensors extends Capability {

    public Sensors(Robot parentRobot) {
        super(parentRobot);
    }

    void init(HardwareMap hardwareMap) {
    }

    @Override
    public void run() {

    }

    public void teleOp(Gamepad gamepad1, Gamepad gamepad2) {}
}
