package org.firstinspires.ftc.teamcode.common;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class QPolynomial {
    //ax^5 + bx^4 + cx^3 + dx^2 + ex + f
    private final double a, b, c, d, e, f, lX, rX;
    /*
    a(s.x)^5  +  b(s.x)^4  +  c(s.x)^3  +  d(s.x)^2  +  e(s.x)  +  f  =  s.y
                5a(s.x)^4  + 4b(s.x)^3  + 3c(s.x)^2  + 2d(s.x)  +  e  =  dS
                            20a(s.x)^3  +12b(s.x)^2  + 6c(s.x)  + 2d  =  d2S
    s
    a(e.x)^5  +  b(e.x)^4  +  c(e.x)^3  +  d(e.x)^2  +  e(e.x)  +  f  =  e.y
                5a(e.x)^4  + 4b(e.x)^3  + 3c(e.x)^2  + 2d(e.x)  +  e  =  dE
                            20a(e.x)^3  +12b(e.x)^2  + 6c(e.x)  + 2d  =  d2E
     */


    public QPolynomial(VectorD start, double dStart, double d2Start, VectorD end, double dEnd,
                       double d2End) throws IllegalArgumentException{
        if(end.getX() <= start.getX()) throw new IllegalArgumentException("endpoint must have a larger x value");
        lX = start.getX();
        rX = end.getX();

        double s1 = start.getX();
        double s2 = s1*s1;
        double s3 = s2*s1;
        double s4 = s3*s1;
        double s5 = s4*s1;

        double e1 = end.getX();
        double e2 = e1*e1;
        double e3 = e2*e1;
        double e4 = e3*e1;
        double e5 = e4*e1;

        RealMatrix coefficients = new Array2DRowRealMatrix(new double[][] {
                {s5   , s4   , s3  , s2  , s1  , 1},
                {5*s4 , 4*s3 , 3*s2, 2*s1, 1   , 0},
                {20*s3, 12*s2, 6*s1, 2   , 0   , 0},
                {e5   , e4   , e3  , e2  , e1  , 1},
                {5*e4 , 4*e3 , 3*e2, 2*e1, 1   , 0},
                {20*e3, 12*e2, 6*e1, 2   , 0   , 0}
        }, false);

        DecompositionSolver solver = new LUDecomposition(coefficients).getSolver();

        RealVector constants = new ArrayRealVector(new double[] { start.getY(), dStart, d2Start, end.getY(), dEnd, d2End }, false);
        RealVector solution = solver.solve(constants);
        a = solution.getEntry(0);
        b = solution.getEntry(1);
        c = solution.getEntry(2);
        d = solution.getEntry(3);
        e = solution.getEntry(4);
        f = solution.getEntry(5);
    }

    /*
      c(s.x)^3  +  d(s.x)^2  +  e(s.x)  +  f  =  s.y
     3c(s.x)^2  + 2d(s.x)^2  +  e             =  dS

     c(e.x)^3  +  d(e.x)^2  +  e(e.x)  +  f  =  e.y
    4b(e.x)^3  + 3c(e.x)^2  + 2d(e.x)  +  e  =  dE
    0a(e.x)^3  +12b(e.x)^2  + 6c(e.x)  + 2d  =  d2E
     */


    public QPolynomial(double sX, double eX, double a, double b, double c, double d, double e, double f) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        this.e = e;
        this.f = f;
        lX = sX;
        rX = eX;
    }

    //ax^5 + bx^4 + cx^3 + dx^2 + ex + f
    public double f(double x) {
        return a*x*x*x*x*x + b*x*x*x*x + c*x*x*x + d*x*x + e*x + f;
    }

    //5ax^4 + 4bx^3 + 3cx^2 + 2dx + e
    public double d1(double x) {
        return 5*a*x*x*x*x + 4*b*x*x*x + 3*c*x*x + 2*d*x + e;
    }

    //20ax^3 + 12bx^2 + 6cx + 2d
    public double d2(double x) {
        return 20*a*x*x*x + 12*b*x*x + 6*c*x + 2*d;
    }

    //60ax^2 + 24bx + 6c
    public double d3(double x) {
        return 60*a*x*x + 24*b*x + 6*c;
    }

    public double d4(double x) {
        return 120*a*x + 24*b;
    }

    public double d5(double x) {
        return 120*a;
    }

    public double lF() {
        return f(lX);
    }

    public double rF() {
        return f(rX);
    }

    public double ld1() {
        return d1(lX);
    }

    public double rd1() {
        return d1(rX);
    }

    public double ld2() {
        return d2(lX);
    }

    public double rd2() {
        return d2(rX);
    }
}