package org.firstinspires.ftc.teamcode.RobotComponents.Constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Environment;
import org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Obstacle;

@Config
public abstract class FieldConstants {
    public static final Obstacle BRIDGE = new Obstacle(new VectorF[] {
            new VectorF(63, 48),
            new VectorF(63, 72),
            new VectorF(81, 72),
            new VectorF(81, 48)}, 3, 3, 1);

    public static final Obstacle WALL_PARKED_ROBOT = new Obstacle(new VectorF[] {
            new VectorF(63, 0),
            new VectorF(63, 18),
            new VectorF(81, 18),
            new VectorF(81, 0)}, 3, 3, 1);

    public static double x1 = -15;
    public static double y1 = 24;
    public static double x2 = 0;
    public static double y2 = 42;
    public static double x3 = -12;
    public static double y3 = 48;

    public static double maxRepelDist = 14;
    public static double maxRepelPow = 28;

    public static final Obstacle TEST_OBSTACLE = new Obstacle(new VectorF[] {
            new VectorF((float)x1, (float)y1),
            new VectorF((float)x2, (float)y2),
            new VectorF((float)x3, (float)y3)}, 13, maxRepelDist, maxRepelPow);

    public static final Environment TEST = new Environment(new Obstacle[] {TEST_OBSTACLE});



    public static final Environment SKYSTONE_FIELD =
            new Environment(new Obstacle[] {BRIDGE, WALL_PARKED_ROBOT});

    public static final Environment EMPTY_FIELD = new Environment(new Obstacle[] {});
}
