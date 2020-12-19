package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Paths;

import org.firstinspires.ftc.teamcode.Common.VectorD;

class LineSegment extends PolynomialSegment {
    public LineSegment(VectorD start, VectorD end) {
        super(start, end.subtracted(start), end, end.subtracted(start));
    }
}
