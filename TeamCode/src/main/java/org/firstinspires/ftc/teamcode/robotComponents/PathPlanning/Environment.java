package org.firstinspires.ftc.teamcode.robotComponents.PathPlanning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.common.Polygon;
import org.firstinspires.ftc.teamcode.common.VectorD;

public class Environment {
    private Obstacle[] obstacles;

    public Environment(Obstacle[] obstacles) {
        this.obstacles = obstacles;
    }

    public VectorD repel(Polygon robot) {
        VectorD repel = new VectorD(0, 0);
        for(Obstacle o : obstacles) {
            repel.add(o.repel(robot));
        }
        return repel;
    }

    public VectorD repel(VectorD robot) {
        VectorD repel = new VectorD(0, 0);
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
