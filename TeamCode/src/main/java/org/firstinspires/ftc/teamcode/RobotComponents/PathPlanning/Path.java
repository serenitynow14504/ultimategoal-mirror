package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning;

import android.util.Pair;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


import org.firstinspires.ftc.teamcode.Common.Line;
import org.firstinspires.ftc.teamcode.Common.Utilities;
import org.firstinspires.ftc.teamcode.Common.VectorD;


public class Path {
    private VectorD[] points;

    private Line[] segments;

    private double pathLength = 0;

    private VectorD currentRobotPos;
    private Pair<Pair<VectorD, Double>, Line> closestData;

    public double getClosestDist() {
        return closestData.first.second;
    }

    public VectorD getClosestPoint() {
        return closestData.first.first;
    }

    public Line getClosestLine() {
        return closestData.second;
    }

    public int getLeftRight() {
        return getClosestLine().calculateLeftRight(currentRobotPos);
    }

    public double getNormalError() {
        return getClosestDist() * getLeftRight();
    }



    public Path(VectorD[] points) {
        this.points = points;
        segments = new Line[points.length - 1];
        for(int i = 1; i < points.length; i++) {
            segments[i-1] = new Line(points[i-1], points[i], i-1);
        }

        for(Line segment: segments){
            pathLength += segment.getLength();
        }
    }

    public double[] getXs() {
        return Utilities.extractXs(points);
    }

    public double[] getYs() {
        return Utilities.extractYs(points);
    }


    public void calculateClosestData(VectorD point) {
        currentRobotPos = point;
        Pair<VectorD, Double> closest = new Pair<>(segments[0].getPointA(), Utilities.distance(point, segments[0].getPointA()));
        Line segment = segments[0];
        for(Line l : segments) {
            VectorD indClosest = l.getClosestPoint(point).first;
            double dist = Utilities.distance(point, indClosest);
            if(dist <= closest.second) {
                closest = new Pair<>(indClosest, dist);
                segment = l;
            }
        }

        closestData = new Pair<>(closest, segment);
    }

    public double closestParameter() {
        Pair<Pair<VectorD, Double>, Line> closestPoint = closestData;
        double parameter = getClosestLine().getParameter(getClosestPoint());
        for(int i = 0; i < getClosestLine().getPathPos(); i++) {
            parameter += segments[i].getLength();
        }
        return parameter;
    }

    public VectorD pointFromParameter(double parameter) {
        for(Line l : segments) {
            if(parameter <= l.getLength()) {
                return l.getParamatrizedPoint(parameter);
            }
            parameter -= l.getLength();
        }
        return segments[segments.length-1].getPointB();
    }


    /*static Line[] pointsToSegments(VectorD[] points) {

    }*/

    public double getPathLength() {
        return pathLength;
    }

    public double parameterFromPoint(VectorD point) {
        calculateClosestData(point);
        return closestParameter();
    }

    public VectorD getPointFromArray(int i) {
        return points[i];
    }


    public VectorD getLastPoint() {
        return pointFromParameter(getPathLength());
    }

    public VectorD getFirstPoint() {
        return  pointFromParameter(0);
    }

    public void flipForBlue() {
        //points[0] = new VectorD(-points[0].get(0), points[0].get(1));
        if(points[1].length() == 2) {
            for (int i = 1; i < points.length; i++) {
                points[i] = new VectorD(-points[i].get(0), points[i].get(1));
                segments[i - 1] = new Line(points[i - 1], points[i], i - 1);
            }
        } else if(points[1].length() == 3) {
            for (int i = 1; i < points.length; i++) {
                points[i] = new VectorD(-points[i].get(0), points[i].get(1), -points[i].get(2));
                segments[i - 1] = new Line(points[i - 1], points[i], i - 1);
            }
        }

        pathLength = 0;
        for(Line segment: segments){
            pathLength += segment.getLength();
        }
    }

    Line getSegmentFromArray(int i) {
        return segments[i];
    }

    public void show(TelemetryPacket packet, String color) {
        packet.fieldOverlay().setStrokeWidth(1).setStroke(color).strokePolyline(getXs(), getYs());
    }
}
