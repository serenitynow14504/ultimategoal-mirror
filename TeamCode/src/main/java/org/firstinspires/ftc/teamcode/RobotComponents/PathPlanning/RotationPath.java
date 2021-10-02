package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning;


import org.firstinspires.ftc.teamcode.Common.Line;
import org.firstinspires.ftc.teamcode.Common.VectorD;

public class RotationPath {
    private VectorD[] points;
    private Line[] segments;

    public RotationPath(Path path, double[] rots) {
        points = new VectorD[rots.length];
        segments = new Line[rots.length-1];
        float dist = 0;
        points[0] = new VectorD(0, (float)rots[0]);
        for(int i = 1; i < rots.length; i++) {
            dist += path.getSegmentFromArray(i-1).getLength();
            points[i] = new VectorD(dist, (float)rots[i]);
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
