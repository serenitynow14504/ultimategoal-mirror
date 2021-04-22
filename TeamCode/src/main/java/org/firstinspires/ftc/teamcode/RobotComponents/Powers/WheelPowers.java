package org.firstinspires.ftc.teamcode.RobotComponents.Powers;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.Common.Util;
import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;

public class WheelPowers {

    private double fl = 0, fls = 0;//y+x-r
    private double fr = 0, frs = 0;//y-x+r
    private double bl = 0, bls = 0;//y-x-r
    private double br = 0, brs = 0;//y+x+r

    private double kS = 0;

    public double getFL() {
        return fls;
    }
    public double getFR() {
        return frs;
    }
    public double getBL() {
        return bls;
    }
    public double getBR() {
        return brs;
    }
    public double getKS() {
        return kS;
    }

    public void set(double fl, double fr, double bl, double br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;

        scale();

        VectorD v = Util.normalize(Util.abs(getLocalVector()));
        kS = v.dotProduct(RobotConstants.KStatics);

        fls = applyStatic(fl, kS);
        frs = applyStatic(fr, kS);
        bls = applyStatic(bl, kS);
        brs = applyStatic(br, kS);
    }

    private double applyStatic(double p, double kStatic) {
        return Math.signum(p)*(kStatic + Math.abs(p)*(1-kStatic));
    }

    public void set(double p) {
        set(p, p, p, p);
    }

    public void setTankStrafe(double l, double r, double s) {
        set(l+s, r-s, l-s, r+s);
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
        setLocalTrans(Util.rotate(v, -Math.toRadians(robotRotDegrees)));
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
        VectorD rotated = Util.rotate(v, -Math.toRadians(robotRotDegrees));
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
