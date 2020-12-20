package org.firstinspires.ftc.teamcode.RobotComponents.Powers;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.Common.Utilities;
import org.firstinspires.ftc.teamcode.Common.VectorD;

public class WheelPowers {

    public double fl = 0;//y+x-r
    public double fr = 0;//y-x+r
    public double bl = 0;//y-x-r
    public double br = 0;//y+x+r

    public void set(double fl, double fr, double bl, double br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;

        scale();
    }

    public void set(double p) {
        set(p, p, p, p);
    }

    public void set(double l, double r) {
        set(l, r, l, r);
    }

    public void scale() {
        double max = Math.max(fl, Math.max(fr, Math.max(bl, br)));
        if(max>1) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }
    }

    public void setLocalTrans(VectorD v) {
        VectorD current = getLocalVector();
        setFromLocalVector(new VectorD(v.getX(), v.getY(), current.getZ()));
    }

    public void setGlobalTrans(VectorD v, double robotRotDegrees) {
        setLocalTrans(Utilities.rotate(v, -Math.toRadians(robotRotDegrees)));
    }

    public void setRotation(double r) {
        VectorD current = getLocalVector();
        setFromLocalVector(new VectorD(current.getX(), current.getY(), r));
    }

    public void setFromLocalVector(VectorD v) {
        set(v.getY() + v.getX() - v.getZ(),
            v.getY() - v.getX() + v.getZ(),
            v.getY() - v.getX() - v.getZ(),
            v.getY() + v.getX() + v.getZ());
    }

    public void setFromGlobalVector(VectorD v, double robotRotDegrees) {
        VectorD rotated = Utilities.rotate(v, -Math.toRadians(robotRotDegrees));
        setFromLocalVector(new VectorD(rotated.getX(), rotated.getY(), v.getZ()));
    }

    VectorD getLocalVector() {
        RealMatrix coefficients = new Array2DRowRealMatrix(new double[][] { { 1, 1, -1 }, { 1, -1, 1 }, { 1, -1, -1 } }, false);
        DecompositionSolver solver = new LUDecomposition(coefficients).getSolver();

        RealVector constants = new ArrayRealVector(new double[] { fl, fr, bl }, false);
        RealVector solution = solver.solve(constants);

        return new VectorD(solution.getEntry(1), solution.getEntry(0), solution.getEntry(2));
    }

}
