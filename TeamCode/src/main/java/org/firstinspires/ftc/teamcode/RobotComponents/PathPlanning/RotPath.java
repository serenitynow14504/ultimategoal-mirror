package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning;

import org.firstinspires.ftc.teamcode.Common.Line;
import org.firstinspires.ftc.teamcode.Common.Utilities;
import org.firstinspires.ftc.teamcode.Common.VectorD;

public class RotPath extends Path {
    private VectorD[] points;
    private Line[] segments;

    public RotPath(VectorD[] ps) {
        super(Utilities.clipToXYs(ps));
        points = new VectorD[ps.length];
        segments = new Line[ps.length-1];
        float dist = 0;
        points[0] = new VectorD(0, ps[0].getZ());
        for(int i = 1; i < ps.length; i++) {
            //dist += path.getSegmentFromArray(i-1).getLength();
            dist += Utilities.distance(ps[i-1], ps[i]);
            points[i] = new VectorD(dist, ps[i].getZ());
            segments[i-1] = new Line(points[i-1], points[i],i-1);
        }
    }

    public double rotFromParam(double param) {
        for(int i = 0; i<segments.length; i++) {
            if(param < points[i+1].get(0)) {
                return segments[i].yFromX(param);
            }
        }
        return points[points.length-1].get(1);
    }
}
