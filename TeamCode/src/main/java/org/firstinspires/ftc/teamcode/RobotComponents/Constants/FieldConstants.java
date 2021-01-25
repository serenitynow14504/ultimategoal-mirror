package org.firstinspires.ftc.teamcode.RobotComponents.Constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Environment;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Obstacle;

@Config
public abstract class FieldConstants {
    public static final Obstacle BRIDGE = new Obstacle(new VectorD[] {
            new VectorD(63, 48),
            new VectorD(63, 72),
            new VectorD(81, 72),
            new VectorD(81, 48)}, 3, 3, 1);

    public static final Obstacle WALL_PARKED_ROBOT = new Obstacle(new VectorD[] {
            new VectorD(63, 0),
            new VectorD(63, 18),
            new VectorD(81, 18),
            new VectorD(81, 0)}, 3, 3, 1);

    public static double x1 = -15;
    public static double y1 = 24;
    public static double x2 = 0;
    public static double y2 = 42;
    public static double x3 = -12;
    public static double y3 = 48;

    public static double maxRepelDist = 14;
    public static double maxRepelPow = 28;

    public static final Obstacle TEST_OBSTACLE = new Obstacle(new VectorD[] {
            new VectorD(x1, y1),
            new VectorD(x2, y2),
            new VectorD(x3, y3)}, 13, maxRepelDist, maxRepelPow);

    public static final Environment TEST = new Environment(new Obstacle[] {TEST_OBSTACLE});



    public static final Environment SKYSTONE_FIELD =
            new Environment(new Obstacle[] {BRIDGE, WALL_PARKED_ROBOT});

    public static final Environment EMPTY_FIELD = new Environment(new Obstacle[] {});

    public static final VectorD TARGET_ZONE_1 = new VectorD(84, 84);
    public static final VectorD TARGET_ZONE_2 = new VectorD(60, 108);
    public static final VectorD TARGET_ZONE_3 = new VectorD(84, 132);

    public static VectorD getTargetZone(int zone) {
        switch(zone) {
            case 1:
                return TARGET_ZONE_2;
            case 2:
                return TARGET_ZONE_3;
            default:
                return TARGET_ZONE_1;
        }
    }

    public static final VectorD HIGH_GOAL = new VectorD(60, 144, 35.5);
    public static final VectorD LEFT_POWER_SHOT = new VectorD(28.5, 144, 23);
    public static final VectorD MID_POWER_SHOT = new VectorD(36, 144, 23);
    public static final VectorD RIGHT_POWER_SHOT = new VectorD(43.5, 144, 23);

    public static final VectorD RING_STACK = new VectorD(60, 48);
    public static final VectorD LEFT_WOBBLE = new VectorD(48, 28);
    public static final VectorD RIGHT_WOBBLE = new VectorD(72, 28);
}
