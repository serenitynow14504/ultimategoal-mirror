package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning;

import org.firstinspires.ftc.teamcode.Common.Utilities;
import org.firstinspires.ftc.teamcode.Common.VectorD;

public class RotPath extends SplinePath {
    Path rotation;
    public RotPath(VectorD[] points) {
        super(Utilities.clipToXYs(points));
        for(VectorD point : points) {
            
        }
    }
}
