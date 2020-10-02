package org.firstinspires.ftc.teamcode.RobotComponents;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors extends Capability {

    private ColorSensor liftColor;
    private DistanceSensor liftDist;
    private DigitalChannel liftMag;
    private RevTouchSensor frontBumper;
    private DistanceSensor dist2;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;

    public Sensors(Robot parentRobot) {
        super(parentRobot);
    }

    void init() {
        liftColor = parent.getMyOpMode().hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        liftDist = parent.getMyOpMode().hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        dist2 = parent.getMyOpMode().hardwareMap.get(DistanceSensor.class, "dist2");
        liftMag = parent.getMyOpMode().hardwareMap.get(DigitalChannel.class,"liftMag");
        frontBumper = parent.getMyOpMode().hardwareMap.get(RevTouchSensor.class, "touch");
    }

    @Override
    public void run() {

        while(parent.getMyOpMode().opModeIsActive() && !parent.getMyOpMode().isStopRequested()) {
            Color.RGBToHSV((int) (liftColor.red() * SCALE_FACTOR),
                    (int) (liftColor.green() * SCALE_FACTOR),
                    (int) (liftColor.blue() * SCALE_FACTOR),
                    hsvValues);
        }
    }

    public float getHue() {
        return hsvValues[0];
    }

    public double getDistRight(DistanceUnit distanceUnit) {
        return liftDist.getDistance(distanceUnit);
    }

    public double getDistLeft(DistanceUnit distanceUnit) {
        return dist2.getDistance(distanceUnit);
    }

    public boolean getLiftSwitchState() {
        return !liftMag.getState();
    }

    boolean getFrontBumperState() {return frontBumper.isPressed();}
}
