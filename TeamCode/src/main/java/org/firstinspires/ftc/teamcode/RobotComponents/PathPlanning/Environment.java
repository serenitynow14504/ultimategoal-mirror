package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Common.Polygon;
import org.firstinspires.ftc.teamcode.Common.VectorD;

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
