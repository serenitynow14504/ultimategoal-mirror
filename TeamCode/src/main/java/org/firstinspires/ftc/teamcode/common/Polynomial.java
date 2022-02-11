package org.firstinspires.ftc.teamcode.common;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class Polynomial {
    private double[] coefficients;//lowest degree to highest
    private PolynomialFunction apacheFunc;

    public Polynomial(double... cs) {
        coefficients = cs;
        apacheFunc = new PolynomialFunction(cs);
    }

    public Polynomial(double sx, double[] sds, double ex, double[] eds) {
        this(calculate(sx, sds, ex, eds));
    }

    private static double[] calculate(double sx, double[] sds, double ex, double[] eds) {
        if(ex <= sx) throw new IllegalArgumentException("endpoint must have a larger x value");

        double[] powsx = new double[sds.length*2];
        powsx[0] = 1;
        for(int i = 1; i<powsx.length; i++) {
            powsx[i] = powsx[i-1] * sx;
        }
        Polynomial sp = new Polynomial(powsx);

        double[] powex = new double[sds.length*2];
        powex[0] = 1;
        for(int i = 1; i<powex.length; i++) {
            powex[i] = powex[i-1] * ex;
        }
        Polynomial ep = new Polynomial(powex);

        double[][] arr = new double[powsx.length][powsx.length];
        for(int i = 0; i<arr.length/2; i++) {
            arr[i] = Util.prepend(sp.getCoefficients(), i, 0);
            arr[i+arr.length/2] = Util.prepend(ep.getCoefficients(), i, 0);
            sp = sp.derivative();
            ep = ep.derivative();
        }

        RealMatrix m = new Array2DRowRealMatrix(arr, false);
        DecompositionSolver solver = new LUDecomposition(m).getSolver();

        RealVector ans = solver.solve(new ArrayRealVector(Util.concat(sds, eds)));
        return ans.toArray();
    }

    public int degree() {
        return coefficients.length-1;
    }

    public double[] getCoefficients() {
        return coefficients;
    }

    public double f(double t) {
        double ans = coefficients[0], pow = 1;
        for(int i = 0; i<degree(); i++) {
            pow *= t;
            ans += pow*coefficients[i+1];
        }
        return ans;
    }

    public Polynomial derivative() {
        double[] csn = new double[degree()];
        for(int i = 1; i<=degree(); i++) {
            csn[i-1] = coefficients[i]*i;
        }
        return new Polynomial(csn);
    }

    public Polynomial squared() {
        double[] csn = new double[degree()*2 + 1];
        for(int i = 0; i<=degree(); i++) {
            for(int j = 0; j<=degree(); j++) {
                csn[i+j] += coefficients[i]*coefficients[j];
            }
        }
        return new Polynomial(csn);
    }


    public Polynomial scale(double fac) {
        double[] csn = new double[degree()+1];
        for(int i = 0; i<degree(); i++) {
            csn[i] = coefficients[i] * fac;
        }
        return new Polynomial(csn);
    }

    public Polynomial add(Polynomial other) {
        int maxDegree = Math.max(other.degree(), degree());
        double[] csn = new double[maxDegree];
        for(int i = 1; i<=degree(); i++) {
            csn[i-1] = coefficients[i]*i;
        }
        return new Polynomial(csn);
    }

    PolynomialFunction toApacheFunction() {
        return new PolynomialFunction(coefficients);
    }

}