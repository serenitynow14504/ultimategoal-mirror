package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Common.Polygon;
import org.firstinspires.ftc.teamcode.Common.Utilities;
import org.firstinspires.ftc.teamcode.Common.VectorD;

public class Obstacle {
    private double safetyDist, maxRepelDist, maxRepelPow;
    private Polygon shape;
    public Obstacle(VectorD[] vertices, double safetyDist, double maxRepelDist, double maxRepelPow) {
        this.safetyDist = safetyDist;
        shape = new Polygon(vertices);
        this.maxRepelDist = maxRepelDist;
        this.maxRepelPow = maxRepelPow;
    }

    public VectorD repel(Polygon robot) {
        VectorD raw = shape.closestVector(robot);
        VectorD repel;
        if(raw.magnitude() < maxRepelDist) {
            double mag = maxRepelPow - raw.magnitude() * maxRepelPow / maxRepelDist;
            repel = Utilities.setMagnitude(raw, mag);
        } else {
            repel = new VectorD(0, 0);
        }
        return repel;
    }

    public VectorD repel(VectorD robot) {
        VectorD raw = shape.closestVector(robot);
        double newMag = raw.magnitude() - safetyDist;
        if(newMag<0) {
            newMag = 0.01;
        }
        raw = Utilities.setMagnitude(raw, newMag);
        VectorD repel;
        if(raw.magnitude() < maxRepelDist) {
            double mag = maxRepelPow - raw.magnitude() * maxRepelPow / maxRepelDist;
            repel = Utilities.setMagnitude(raw, mag);
        } else {
            repel = new VectorD(0, 0);
        }
        return repel;
    }

    public void show(TelemetryPacket packet, String color) {
        shape.show(packet, color);
    }
}
