package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


import org.firstinspires.ftc.robotcore.internal.android.dex.util.ExceptionWithContext;

import static org.firstinspires.ftc.teamcode.common.Util.*;

public class Polygon {
    private VectorD[] vertices;
    private Line[] sides;

    public Polygon(VectorD[] vertices) throws ExceptionWithContext{
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

    public VectorD closestVector(VectorD point) {
        VectorD[] lengths = new VectorD[vertices.length];

        for(int i = 0; i < lengths.length; i++) {
            lengths[i] = sides[i].closestVector(point);
        }

        return minLengthVector(lengths);
    }

    public VectorD closestVector(Line line) {
        VectorD[] lengths = new VectorD[vertices.length];

        for(int i = 0; i < lengths.length; i++) {
            lengths[i] = sides[i].closestVector(line);
        }

        return minLengthVector(lengths);
    }

    public VectorD closestVector(Polygon polygon) {
        VectorD[] lengths = new VectorD[sides() * polygon.sides()];

        for(int i = 0; i < sides(); i++) {
            for(int j = 0; j < polygon.sides(); j++) {
                lengths[i*polygon.sides() + j] = sides[i].closestVector(polygon.getSideInArray(j));
                //lengths[i] = polygon.getSideInArray(i).closestVector(sides[j]);
            }
        }

        return minLengthVector(lengths).multiplied(1.0);
    }

    public void show(TelemetryPacket packet, String color) {
        double[] newXs = extractYs(vertices);
        for(int i = 0; i<newXs.length; i++) {
            newXs[i]-=72;
        }
        double[] newYs = extractXs(vertices);
        for(int i = 0; i<newYs.length; i++) {
            newYs[i]*=-1d;
            newYs[i]+=24;
        }
        packet.fieldOverlay().setStroke(color).strokePolygon(newXs, newYs);
    }
}
