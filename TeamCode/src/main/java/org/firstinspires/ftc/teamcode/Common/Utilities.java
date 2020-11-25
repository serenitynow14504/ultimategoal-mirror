package org.firstinspires.ftc.teamcode.Common;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.internal.android.dex.util.ExceptionWithContext;

public class Utilities {
    public static double distance(VectorD a, VectorD b) {
        return Math.sqrt((b.get(0)-a.get(0))*(b.get(0)-a.get(0)) + (b.get(1)-a.get(1))*(b.get(1)-a.get(1)));
    }

    public static double[] extractYs(VectorD[] v) {
        double[] doobles = new double[v.length];
        for(int i = 0; i < v.length; i++) {
            doobles[i] = v[i].get(1);
        }
        return doobles;
    }

    public static double[] extractXs(VectorD[] v) {
        double[] doobles = new double[v.length];
        for(int i = 0; i < v.length; i++) {
            doobles[i] = v[i].get(0);
        }
        return doobles;
    }

    public static double[] extractZs(VectorD[] v) {
        double[] doobles = new double[v.length];
        for(int i = 0; i < v.length; i++) {
            doobles[i] = v[i].get(2);
        }
        return doobles;
    }

    public static VectorD[] clipToXYs(VectorD[] v) {
        VectorD[] vectors = new VectorD[v.length];
        for(int i = 0; i < v.length; i++) {
            vectors[i] = new VectorD(v[i].get(0), v[i].get(1));
        }
        return vectors;
    }



    public static VectorD[] combineXY(double[] xs, double[] ys) {
        if(xs.length != ys.length) {
            throw new ExceptionWithContext("arrays are different sizes");
        }
        VectorD[] vectors = new VectorD[xs.length];
        for(int i = 0; i < xs.length; i++) {
            vectors[i] = new VectorD((float)xs[i], (float)ys[i]);
        }
        return vectors;
    }

    public static VectorD minLengthVector(VectorD[] vectors) {
        VectorD min = vectors[0];
        for(int i = 0; i < vectors.length; i++) {
            if(vectors[i].magnitude() < min.magnitude()) {
                min = vectors[i];
            }
        }
        return min;
    }

    public static VectorD rotate (VectorD v, double r) {
        return new VectorD(v.get(0) * (float)Math.cos(r) - v.get(1) * (float)Math.sin(r),
                v.get(1) * (float)Math.cos(r) + v.get(0) * (float)Math.sin(r));
    }

    public static VectorD clipToXY(VectorD v) {
        return new VectorD(v.get(0), v.get(1));
    }

    public static VectorD normalize(VectorD v) {
        if(v.magnitude() == 0) return duplicate(v);
        return v.multiplied((double)1/v.magnitude());
    }

    public static VectorD setMagnitude(VectorD v, double mag) {
        return normalize(v).multiplied(mag);
    }

    public static boolean isEqual(VectorD a, VectorD b) {
        if(a.length() != b.length()) {
            throw new IllegalArgumentException("vectors are of different lengths");
        }
        boolean bool = a.get(0) == b.get(0) && a.get(1) == b.get(1);
        if (a.length() == 3) {
            bool = bool && a.get(2) == b.get(2);
        }
        return bool;
    }

    public static void log(String s) {
        RobotLog.d("Bruh " + s);
    }

    public static void log(double s) {RobotLog.d("Bruh " + s);}

    public static VectorD duplicate(VectorD v) {
        if(v.length() == 2) {
            return new VectorD(v.get(0), v.get(1));
        } else {
            return new VectorD(v.get(0), v.get(1), v.get(2));
        }
    }

    public static void drawArrow(TelemetryPacket packet, VectorD a, VectorD b, String color) {
        double dist = distance(a, b);
        if(dist != 0) {
            packet.fieldOverlay().setStroke(color).strokeLine(a.get(0), a.get(1), b.get(0), b.get(1));
            //dist /= 6;
            VectorD a1 = setMagnitude(rotate(a.subtracted(b), Math.PI / 5),
                    3).added(b);
            VectorD a2 =
                    setMagnitude(rotate(a.subtracted(b), Math.PI / -5), 3).added(b);

            packet.fieldOverlay().setStroke(color).setStrokeWidth(1).strokeLine(b.get(0), b.get(1),
                    a1.get(0), a1.get(1));
            packet.fieldOverlay().setStroke(color).setStrokeWidth(1).strokeLine(b.get(0), b.get(1),
                    a2.get(0), a2.get(1));
        }
    }


    public static void drawArc(TelemetryPacket packet, VectorD a, VectorD b,
                               VectorD center, int segments, String color) {

        if(distance(a, center) != distance(b, center)) {
            //throw new IllegalArgumentException("a and b are not equidistant from center");
        }

        VectorD localA = a.subtracted(center);

        double dist = distance(a, b);
        if(dist != 0) {
            double r = distance(a, center);
            double[] Xs = new double[segments+1];
            double[] Ys = new double[segments+1];

            double centralAng = Math.atan2(a.subtracted(center).get(1), a.subtracted(center).get(0)) -
                    Math.atan2(b.subtracted(center).get(1), b.subtracted(center).get(0));
            double segmentAng = centralAng / segments;

            for(int i = 0; i < segments; i++) {
                VectorD point = rotate(localA, i*segmentAng).added(center);
                Xs[i] = point.get(0);
                Ys[i] = point.get(1);
            }
            Xs[segments] = b.get(0);
            Ys[segments] = b.get(1);

            packet.fieldOverlay().setStroke(color).strokePolyline(Xs, Ys);
        }

        packet.fieldOverlay().setStroke(color).setFill(color).fillCircle(center.get(0),
                center.get(1), 1.5);
    }

    public static VectorD[] convertToGlobalLine(VectorD a, VectorD b, VectorD pos) {
        VectorD globalA = rotate(a, pos.get(2)).added(clipToXY(pos));
        VectorD globalB = rotate(b, pos.get(2)).added(clipToXY(pos));
        return new VectorD[]{globalA, globalB};
    }

    public static void drawLocalArrow(TelemetryPacket packet, VectorD a, VectorD b,
                                      VectorD pos, String color) {
        VectorD[] globals = convertToGlobalLine(a, b, pos);
        drawArrow(packet, globals[0], globals[1], color);
    }

    public static void drawLocalLine(TelemetryPacket packet, VectorD a, VectorD b,
                                      VectorD pos, String color) {
        VectorD[] globals = convertToGlobalLine(a, b, pos);
        packet.fieldOverlay().setStroke(color).strokeLine(globals[0].get(0), globals[0].get(1),
                globals[1].get(0), globals[1].get(1));
    }
}
