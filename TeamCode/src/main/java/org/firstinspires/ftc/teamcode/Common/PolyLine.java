package org.firstinspires.ftc.teamcode.Common;

public class PolyLine {
    private VectorD[] points;
    private Line[] segments;
    public PolyLine(VectorD[] ps) {
        //input x coords HAVE to be in ascending order
        points = ps;
        segments = new Line[ps.length-1];

        for(int i = 1; i < ps.length; i++) {
            segments[i-1] = new Line(points[i-1], points[i],i-1);
        }

    }

    public double yFromX(double x) {
        for(int i = 0; i<segments.length; i++) {
            if(x>=points[i].getX()) {
                return segments[i].yFromX(x);
            }
        }
        return segments[segments.length-1].yFromX(x);
    }
}
