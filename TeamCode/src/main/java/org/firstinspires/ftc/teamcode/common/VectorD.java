package org.firstinspires.ftc.teamcode.common;

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

    public double getZ() {
        return get(2);
    }

    public boolean hasZ() {
        return length()>2;
    }


    @Const
    public VectorD multiplied(double scale)
    {
        VectorF result = multiplied((float)scale);
        if(result.length()==2) {
            return new VectorD(result.get(0), result.get(1));
        } else {
            return new VectorD(result.get(0), result.get(1), result.get(2));
        }
    }

    @NonConst
    public void multiply(double scale)
    {
        multiply((float)scale);
    }

    @Const
    public VectorD added(VectorD addend)
    {
        VectorF result = added((VectorF)addend);
        if(result.length()==2) {
            return new VectorD(result.get(0), result.get(1));
        } else {
            return new VectorD(result.get(0), result.get(1), result.get(2));
        }
    }

    @NonConst
    public void add(VectorD addend)
    {
        add((VectorF)addend);
    }

    @Const
    public VectorD subtracted(VectorD subtrahend)
    {
        VectorF result = subtracted((VectorF)subtrahend);
        if(result.length()==2) {
            return new VectorD(result.get(0), result.get(1));
        } else {
            return new VectorD(result.get(0), result.get(1), result.get(2));
        }
    }

    @NonConst
    public void subtract(VectorD subtrahend)
    {
        subtract((VectorF)subtrahend);
    }

//    public double dotProduct(VectorD other) {
//        return super.dotProduct(other);
//    }
}
