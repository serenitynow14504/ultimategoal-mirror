package org.firstinspires.ftc.teamcode.robotComponents.PathPlanning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.common.Line;
import org.firstinspires.ftc.teamcode.common.Util;
import org.firstinspires.ftc.teamcode.common.VectorD;



public class Path {
    private VectorD[] transPoints;
    private Line[] transSegments;

    private VectorD[] rotPoints;
    private Line[] rotSegments;

    private double pathLength = 0;

    private VectorD currentRobotPos;
    private VectorD closestPathPoint;
    private double closestDist;
    private Line closestTransSegment;

    public double getClosestDist() {
        return closestDist;
    }

    public VectorD getClosestPoint() {
        return closestPathPoint;
    }

    public Line getClosestLine() {
        return closestTransSegment;
    }

    public int getLeftRight() {
        return getClosestLine().calculateLeftRight(currentRobotPos);
    }

    public double getNormalError() {
        return getClosestDist() * getLeftRight();
    }



    public Path(VectorD[] points) {
        this.transPoints = Util.clipToXYs(points);
        transSegments = new Line[points.length - 1];
        for(int i = 1; i < points.length; i++) {
            transSegments[i-1] = new Line(points[i-1], points[i], i-1);
        }
        for(Line segment: transSegments) {
            pathLength += segment.getLength();
        }


        rotPoints = new VectorD[points.length];
        rotSegments = new Line[points.length-1];
        float dist = 0;
        rotPoints[0] = new VectorD(0, points[0].getZ());
        for(int i = 1; i < points.length; i++) {
            dist += Util.distance(points[i-1], points[i]);
            if(points[i].hasZ()) {
                rotPoints[i] = new VectorD(dist, points[i].getZ());
                rotSegments[i - 1] = new Line(rotPoints[i - 1], rotPoints[i], i - 1);
            }
        }
    }


    /**
     * for use by json path loader
     * @param translationPts points that define translation polyline
     * @param rotationPts points that define rotation over distance along path (dist, rot)
     */
    public Path(VectorD[] translationPts, VectorD[] rotationPts) {
        this.transPoints = translationPts;
        transSegments = new Line[translationPts.length - 1];
        for(int i = 1; i < translationPts.length; i++) {
            transSegments[i-1] = new Line(translationPts[i-1], translationPts[i], i-1);
        }
        for(Line segment: transSegments) {
            pathLength += segment.getLength();
        }

        rotPoints = rotationPts;
        rotSegments = new Line[rotPoints.length-1];
        for(int i = 1; i < rotPoints.length; i++) {
            rotSegments[i-1] = new Line(rotPoints[i-1], rotPoints[i],i-1);
        }
    }

    public double[] getXs() {
        return Util.extractXs(transPoints);
    }

    public double[] getYs() {
        return Util.extractYs(transPoints);
    }


    public void calculateClosestData(VectorD point) {
        currentRobotPos = point;

        VectorD cp = transSegments[0].getPointA();
        double cd = Util.distance(point, transSegments[0].getPointA());
        Line segment = transSegments[0];

        for(Line l : transSegments) {
            VectorD indClosest = l.getClosestPoint(point).first;
            double dist = Util.distance(point, indClosest);
            if(dist <= cd) {
                cp = indClosest;
                cd = dist;
                segment = l;
            }
        }

        closestPathPoint = cp;
        closestDist = cd;
        closestTransSegment = segment;
    }

    public double closestParameter() {
        double parameter = getClosestLine().getParameter(getClosestPoint());
        for(int i = 0; i < getClosestLine().getPathPos(); i++) {
            parameter += transSegments[i].getLength();
        }
        return parameter;
    }

    public VectorD pointFromParameter(double parameter) {
        for(Line l : transSegments) {
            if(parameter <= l.getLength()) {
                return l.getParamatrizedPoint(parameter);
            }
            parameter -= l.getLength();
        }
        return transSegments[transSegments.length-1].getPointB();
    }

    public double rotFromParam(double param) {
        for(int i = 0; i< rotSegments.length; i++) {
            if(param < rotPoints[i+1].get(0)) {
                return rotSegments[i].yFromX(param);
            }
        }
        return rotPoints[rotPoints.length-1].get(1);
    }
    public double getPathLength() {
        return pathLength;
    }

    public double parameterFromPoint(VectorD point) {
        calculateClosestData(point);
        return closestParameter();
    }

    public VectorD getPointFromArray(int i) {
        return transPoints[i];
    }


    public VectorD getLastPoint() {
        return transPoints[transPoints.length-1];
        //return pointFromParameter(getPathLength());
    }

    public VectorD getFirstPoint() {
        return  pointFromParameter(0);
    }

    public void flipForBlue() {
        //points[0] = new VectorD(-points[0].get(0), points[0].get(1));
        if(transPoints[1].length() == 2) {
            for (int i = 1; i < transPoints.length; i++) {
                transPoints[i] = new VectorD(-transPoints[i].get(0), transPoints[i].get(1));
                transSegments[i - 1] = new Line(transPoints[i - 1], transPoints[i], i - 1);
            }
        } else if(transPoints[1].length() == 3) {
            for (int i = 1; i < transPoints.length; i++) {
                transPoints[i] = new VectorD(-transPoints[i].get(0), transPoints[i].get(1), -transPoints[i].get(2));
                transSegments[i - 1] = new Line(transPoints[i - 1], transPoints[i], i - 1);
            }
        }

        pathLength = 0;
        for(Line segment: transSegments){
            pathLength += segment.getLength();
        }
    }

    Line getSegmentFromArray(int i) {
        return transSegments[i];
    }

    public void show(TelemetryPacket packet, String color) {
        double[] newXs = getYs();
        for(int i = 0; i<newXs.length; i++) {
            newXs[i]-=72;
        }
        double[] newYs = getXs();
        for(int i = 0; i<newYs.length; i++) {
            newYs[i]*=-1d;
            newYs[i]+=24;
        }
        packet.fieldOverlay().setStrokeWidth(1).setStroke(color).strokePolyline(newXs, newYs);
    }
}
