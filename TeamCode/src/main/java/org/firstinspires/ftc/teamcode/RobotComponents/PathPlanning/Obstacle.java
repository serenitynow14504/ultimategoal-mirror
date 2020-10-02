package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Common.Polygon;
import org.firstinspires.ftc.teamcode.Common.Utilities;

public class Obstacle {
    private double safetyDist, maxRepelDist, maxRepelPow;
    private Polygon shape;
    public Obstacle(VectorF[] vertices, double safetyDist, double maxRepelDist, double maxRepelPow) {
        this.safetyDist = safetyDist;
        shape = new Polygon(vertices);
        this.maxRepelDist = maxRepelDist;
        this.maxRepelPow = maxRepelPow;
    }

    public VectorF repel(Polygon robot) {
        VectorF raw = shape.closestVector(robot);
        VectorF repel;
        if(raw.magnitude() < maxRepelDist) {
            double mag = maxRepelPow - raw.magnitude() * maxRepelPow / maxRepelDist;
            repel = Utilities.setMagnitude(raw, mag);
        } else {
            repel = new VectorF(0, 0);
        }
        return repel;
    }

    public VectorF repel(VectorF robot) {
        VectorF raw = shape.closestVector(robot);
        double newMag = raw.magnitude() - safetyDist;
        if(newMag<0) {
            newMag = 0.01;
        }
        raw = Utilities.setMagnitude(raw, newMag);
        VectorF repel;
        if(raw.magnitude() < maxRepelDist) {
            double mag = maxRepelPow - raw.magnitude() * maxRepelPow / maxRepelDist;
            repel = Utilities.setMagnitude(raw, mag);
        } else {
            repel = new VectorF(0, 0);
        }
        return repel;
    }

    public void show(TelemetryPacket packet, String color) {
        shape.show(packet, color);
    }
}
