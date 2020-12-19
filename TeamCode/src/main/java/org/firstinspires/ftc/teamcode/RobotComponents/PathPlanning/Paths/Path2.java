package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Paths;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Common.VectorD;

import java.util.ArrayList;

public class Path2 {
    VectorD start, startHandle = null;
    boolean finishedBuilding = false;

    ArrayList<PolynomialSegment> segments;
    ArrayList<PolynomialSegment> rotations;

    public Path2(VectorD s) {
        segments = new ArrayList<>();
        rotations = new ArrayList<>();
        start = s;
    }

    public Path2(VectorD s, VectorD sH) {
        segments = new ArrayList<>();
        rotations = new ArrayList<>();
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

    private PolynomialSegment getLastSegment() {
        if(segments.size()==0) return null;
        return segments.get(segments.size()-1);
    }

    public Path2 addLine(VectorD p) throws Exception{
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

    public Path2 addSpline(VectorD p) throws Exception {
        if(finishedBuilding) throw new IllegalStateException("path already finished");
        if(segments.size() == 0 && startHandle == null) throw new IllegalStateException("Path has to be initialized with a handle if the first segment is a spline");

        if(segments.size() == 0) {
            segments.add(new PolynomialSegment(start, startHandle, p));
        } else if(getLastSegment() instanceof LineSegment) {
            segments.add(new PolynomialSegment(getLastPos(), getLastHandle(), p));
        } else {
            segments.add(new PolynomialSegment(getLastPos(), p.subtracted(getLastPos()), p));
        }

        return this;
    }

    public Path2 finish() throws Exception {
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

    //EXAMPLE PATH CREATION:--------------------------------------------------------------------------------------------------------------
    void asdf() {
        try {
            Path2 test = (new Path2(new VectorD(0, 0), new VectorD(1, 1)))
                    .addSpline(new VectorD(0, 24))
                    .addSpline(new VectorD(24, 0))
                    .addLine(new VectorD(24, 24))
                    .finish();
        } catch (Exception e) {
            RobotLog.d("Bruh " + e.getMessage());
        }
    }

    //Reparamatrization===================================================================================================================

    double paramatrize(double length) {
        return length;
    }



    public VectorD vel(double param) {
        return new VectorD(0, 0);
    }



}
