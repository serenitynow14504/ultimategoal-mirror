package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Common.Polygon;

public class Environment {
    private Obstacle[] obstacles;

    public Environment(Obstacle[] obstacles) {
        this.obstacles = obstacles;
    }

    public VectorF repel(Polygon robot) {
        VectorF repel = new VectorF(0, 0);
        for(Obstacle o : obstacles) {
            repel.add(o.repel(robot));
        }
        return repel;
    }

    public VectorF repel(VectorF robot) {
        VectorF repel = new VectorF(0, 0);
        for(Obstacle o : obstacles) {
            repel.add(o.repel(robot));
        }
        return repel;
    }

    public void show(TelemetryPacket packet, String color) {
        for(Obstacle o : obstacles) {
            o.show(packet, color);
        }
    }
}
