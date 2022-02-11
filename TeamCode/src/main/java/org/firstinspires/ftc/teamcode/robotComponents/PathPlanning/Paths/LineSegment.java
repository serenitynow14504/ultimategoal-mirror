package org.firstinspires.ftc.teamcode.robotComponents.PathPlanning.Paths;

import org.firstinspires.ftc.teamcode.common.PolynomialPathSegment;
import org.firstinspires.ftc.teamcode.common.VectorD;

class LineSegment extends PolynomialPathSegment {
    public LineSegment(VectorD start, VectorD end) {
        super(start, end.subtracted(start), end, end.subtracted(start));
    }
}
