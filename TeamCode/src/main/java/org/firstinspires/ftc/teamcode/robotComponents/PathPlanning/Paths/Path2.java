package org.firstinspires.ftc.teamcode.robotComponents.PathPlanning.Paths;

import org.firstinspires.ftc.teamcode.common.PolynomialPathSegment;
import org.firstinspires.ftc.teamcode.common.VectorD;
import org.firstinspires.ftc.teamcode.robotComponents.PathPlanning.RotPath;

import java.util.ArrayList;

public class Path2 {
    VectorD start, startHandle = null;
    boolean finishedBuilding = false;

    ArrayList<PolynomialPathSegment> segments;


    //PATH CONSTRUCTION===================================================================================================================
    public Path2(VectorD s) {
        segments = new ArrayList<>();
        start = s;
    }

    public Path2(VectorD s, VectorD sH) {
        segments = new ArrayList<>();
        start = s;
        startHandle = sH;
    }

    private VectorD getLastPos() {
        if(segments.size()==0) return start;
        return segments.get(segments.size()-1).end;
    }

    private VectorD getLastHandle() {
        if(segments.size()==0) return startHandle;
        return segments.get(segments.size()-1).eHandle;
    }

    private PolynomialPathSegment getLastSegment() {
        if(segments.size()==0) return null;
        return segments.get(segments.size()-1);
    }

    public Path2 lineTo(VectorD p) throws RuntimeException {
        if(finishedBuilding) throw new IllegalStateException("path already finished");
        if(startHandle != null) throw new IllegalStateException("Path cannot be initialized with a handle if the first segment is a line");
        if(segments.size()>0 && segments.get(segments.size()-1) instanceof LineSegment) throw new IllegalStateException("Cannot connect 2 line segments together");

        if(segments.size()>0) {
            getLastSegment().finish(p.subtracted(start));
        }
        segments.add(new LineSegment(getLastPos(), p));

        //rotations.add

        return this;
    }

    public Path2 splineTo(VectorD p) throws RuntimeException {
        if(finishedBuilding) throw new IllegalStateException("path already finished");
        if(segments.size() == 0 && startHandle == null) throw new IllegalStateException("Path has to be initialized with a handle if the first segment is a spline");

        if(segments.size() == 0) {
            segments.add(new PolynomialPathSegment(start, startHandle, p));
        } else if(getLastSegment() instanceof LineSegment) {
            segments.add(new PolynomialPathSegment(getLastPos(), getLastHandle(), p));
        } else {
            segments.add(new PolynomialPathSegment(getLastPos(), p.subtracted(getLastPos()), p));
        }

        return this;
    }

    public Path2 finish() throws RuntimeException {
        if(!(getLastSegment() instanceof LineSegment)) {
            throw new IllegalArgumentException("Last segment is a spline; end handle must be specified");
        }
        finishedBuilding = true;
        return this;
    }

    public Path2 finish(VectorD eHandle) {
        getLastSegment().finish(eHandle);
        finishedBuilding = true;
        return this;
    }

    //EXAMPLE PATH CREATION
    private void demo() {
        Path2 test = (new Path2(new VectorD(0, 0), new VectorD(1, 1)))
                .splineTo(new VectorD(0, 24))
                .splineTo(new VectorD(24, 0))
                .lineTo(new VectorD(24, 24))
                .finish();
    }

    //general stuff===================================================================================================================

    public VectorD getTPoint(double t) {
        if(t<0) t = 0;
        if(t>=segments.size()) return segments.get(segments.size()-1).pose(t);
        return segments.get((int)t).pose(t);
    }

    public RotPath toRotPath(int resolutionPerSegment) {
        VectorD[] pPoints = new VectorD[segments.size() * resolutionPerSegment + 1];
        for(int i = 0; i<segments.size(); i++) {
            PolynomialPathSegment seg = segments.get(i);
            for(int j = 0; j<resolutionPerSegment; j++) {
                pPoints[i*resolutionPerSegment + j] = seg.pose((double)j/resolutionPerSegment);
            }
        }
        pPoints[pPoints.length - 1] = segments.get(segments.size() - 1).pose(1);
        return new RotPath(pPoints);
    }

    public RotPath toRotPath() {
        return toRotPath(10);
    }

    //Re-parametrization===================================================================================================================

    double parametrize(double length) {
        return length;
    }



    public VectorD vel(double param) {
        return new VectorD(0, 0);
    }



}
