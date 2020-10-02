package org.firstinspires.ftc.teamcode.Common;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.internal.android.dex.util.ExceptionWithContext;

import static org.firstinspires.ftc.teamcode.Common.Utilities.*;

public class Polygon {
    private VectorF[] vertices;
    private Line[] sides;

    public Polygon(VectorF[] vertices) throws ExceptionWithContext{
        if(vertices.length <= 2) {
            throw new ExceptionWithContext("too little vertices to create polygon");
        }
        this.vertices = vertices;

        sides = new Line[vertices.length];
        for(int i = 0; i<vertices.length; i++) {
            sides[i] = new Line(vertices[i], vertices[(i+1) % vertices.length], i);
        }
    }

    public int sides() {
        return sides.length;
    }

    public Line getSideInArray(int i) {
        if(i<sides()) {
            return sides[i];
        } else {
            return sides[sides()-1];
        }
    }

    public VectorF closestVector(VectorF point) {
        VectorF[] lengths = new VectorF[vertices.length];

        for(int i = 0; i < lengths.length; i++) {
            lengths[i] = sides[i].closestVector(point);
        }

        return minLengthVector(lengths);
    }

    public VectorF closestVector(Line line) {
        VectorF[] lengths = new VectorF[vertices.length];

        for(int i = 0; i < lengths.length; i++) {
            lengths[i] = sides[i].closestVector(line);
        }

        return minLengthVector(lengths);
    }

    public VectorF closestVector(Polygon polygon) {
        VectorF[] lengths = new VectorF[sides() * polygon.sides()];

        for(int i = 0; i < sides(); i++) {
            for(int j = 0; j < polygon.sides(); j++) {
                lengths[i*polygon.sides() + j] = sides[i].closestVector(polygon.getSideInArray(j));
                //lengths[i] = polygon.getSideInArray(i).closestVector(sides[j]);
            }
        }

        return minLengthVector(lengths).multiplied(1);
    }

    public void show(TelemetryPacket packet, String color) {
        packet.fieldOverlay().setStroke(color).strokePolygon(extractXs(vertices),
                extractYs(vertices));
    }
}
