package org.firstinspires.ftc.teamcode.robotComponents.PathPlanning;

import org.firstinspires.ftc.teamcode.common.Util;
import org.firstinspires.ftc.teamcode.common.VectorD;

@Deprecated
public class RotPath extends Path {


    public RotPath(VectorD[] points) {
        super(Util.clipToXYs(points));

    }
}
