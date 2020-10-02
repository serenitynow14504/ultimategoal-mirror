package org.firstinspires.ftc.teamcode.Common;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.NonConst;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class VectorD extends VectorF {
    public VectorD(double x, double y) {
        super((float)x, (float)y);
    }

    public VectorD(double x, double y, double z) {
        super((float)x, (float)y, (float)z);
    }

    public double getX() {
        return get(0);
    }

    public double getY() {
        return get(1);
    }


    @Const
    public VectorF multiplied(double scale)
    {
        return multiplied((float)scale);
    }

    @NonConst
    public void multiply(double scale)
    {
        multiply((float)scale);
    }
}
