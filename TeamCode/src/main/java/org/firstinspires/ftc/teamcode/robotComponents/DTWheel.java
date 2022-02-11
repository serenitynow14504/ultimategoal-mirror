package org.firstinspires.ftc.teamcode.robotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.PIDController;
import org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants;
import org.openftc.revextensions2.ExpansionHubMotor;

@Config
class DTWheel extends Capability {
    public static double K_P = 0.01, K_I = 0, K_D = 0.0025;
    public static double MAXIMUM_MAXIMUM_VELOCITY = 56;

    private ExpansionHubMotor motor;
    private PIDController controller;
    private boolean reverse = false;
    private String name;

    private double calculatedPower = 0;

    private double MAXIMUM_VELOCITY() {
        return 53;//12v
    }

    public DTWheel(Robot r, String name, boolean reverse) {
        super(r);
        this.name = name;
        this.reverse = reverse;
    }

    public void init(HardwareMap hardwareMap) {
        motor = (ExpansionHubMotor) hardwareMap.get(name);
        if(reverse) motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(K_P, K_I, K_D);
        controller.setOutputRange(0, 1);
        controller.setInputRange(0, MAXIMUM_MAXIMUM_VELOCITY);
        controller.enable();

    }

    @Override
    void tick(TelemetryPacket packet) {
        double vel = motor.getVelocity(AngleUnit.RADIANS) * RobotConstants.DRIVE_GEAR_RATIO * RobotConstants.WHEEL_DIAMETER_INCHES / 2;
        double pidOut = controller.performPID(vel) * getReductionCoeff(calculatedPower, 0.4, 250);
        packet.addLine(name.toUpperCase() + "  calculated power: " + calculatedPower + "     velocity: " + vel + "     pid: " + pidOut + "     setpoint: " + controller.getSetpoint());
        motor.setPower(calculatedPower + pidOut);
    }

    @Deprecated
    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setVelocity(double inchesPerSecond) {
        controller.setSetpoint(inchesPerSecond);
        calculatedPower = inchesPerSecond/MAXIMUM_VELOCITY();
    }


    double getReductionCoeff(double inp, double min, double squeeze) {
        return (inp*inp + min/squeeze) / (inp*inp + 1/squeeze);
    }


    @Override
    void teleOp(Gamepad gamepad1, Gamepad gamepad2) {}


}
