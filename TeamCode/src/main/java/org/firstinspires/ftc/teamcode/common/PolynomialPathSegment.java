package org.firstinspires.ftc.teamcode.common;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.BaseAbstractUnivariateIntegrator;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;

import static org.firstinspires.ftc.teamcode.common.Util.distance;
import static org.firstinspires.ftc.teamcode.common.Util.normalize;

public class PolynomialPathSegment {
    BaseAbstractUnivariateIntegrator integrator = new SimpsonIntegrator();

    private Polynomial x, y, r;
    public VectorD sHandle, eHandle;
    boolean created = false;
    public VectorD start, end;

    public PolynomialPathSegment(VectorD a, VectorD b) {
        x = new Polynomial(0, new double[]{a.getX()}, 1, new double[]{b.getX()});
        y = new Polynomial(0, new double[]{a.getY()}, 1, new double[]{b.getY()});
        r = new Polynomial(0, new double[]{a.getZ()}, 1, new double[]{b.getZ()});
        this.start = a;
        this.end = b;
    }

    public PolynomialPathSegment(VectorD start, VectorD sHandle, VectorD end) {
        this.start = start;
        this.end = end;
        this.sHandle = sHandle;
    }

    /**
     * Uses tangent handles to calculate parametric start/end derivatives, sets acc to 0
     * @param start start point
     * @param sHandle start handle
     * @param end end point
     * @param eHandle end handle
     * @param old useless param to differentiate between this and other method.
     */
    public PolynomialPathSegment(VectorD start, VectorD sHandle, VectorD end, VectorD eHandle, boolean old) {
        this.start = start;
        this.end = end;
        this.sHandle = sHandle;
        this.eHandle = eHandle;

        double dist = distance(start, end);
        x = new Polynomial(0, new double[] {start.getX(), normalize(sHandle).multiplied(dist).getX(), 0},
                1, new double[] {end.getX(), normalize(eHandle).multiplied(dist).getX(), 0});
        y = new Polynomial(0, new double[] {start.getY(), normalize(sHandle).multiplied(dist).getY(), 0},
                1, new double[] {end.getY(), normalize(eHandle).multiplied(dist).getY(), 0});
        created = true;
    }

    /**
     * Bezier Curve
     * @param points all control points, minimum length of 2 defines straight line segment
     */
    public PolynomialPathSegment(VectorD... points) {
        double[] xCoeffs = new double[points.length];
        double[] yCoeffs = new double[points.length];
        Polynomial xPoly = new Polynomial(new double[points.length]);
        Polynomial yPoly = new Polynomial(new double[points.length]);
        for(int i = 0; i<points.length-1; i++) {
            //x.toApacheFunction().
        }
    }
    
    public double getLength(double t) {
        PolynomialFunction sum = x.derivative().squared().toApacheFunction().add(y.derivative().squared().toApacheFunction());
        UnivariateFunction func = v -> Math.sqrt(sum.value(v));
        return integrator.integrate(200, func, 0, t);
    }

    public double getLength() {
        return getLength(1);
    }

    public void setDerivatives(VectorD[] ds, VectorD[] de) {
        x = new Polynomial(0, getXs(start, ds), 1, getXs(end, de));
        y = new Polynomial(0, getYs(start, ds), 1, getYs(end, de));
        r = new Polynomial(0, getZs(start, ds), 1, getZs(end, de));
    }

    public VectorD pose(double t) {
        return new VectorD(x.f(t), y.f(t), r.f(t));
    }

    public VectorD vel(double t) {
        return new VectorD(x.derivative().f(t), y.derivative().f(t), r.derivative().f(t));
    }

    public VectorD acc(double t) {
        return new VectorD(x.derivative().derivative().f(t), y.derivative().derivative().f(t), r.derivative().derivative().f(t));
    }


    double[] getXs(VectorD a, VectorD[] arr) {
        double[] out = new double[arr.length + 1];
        out[0] = a.getX();
        for(int i = 0; i<arr.length; i++) {
            out[i+1] = arr[i].getX();
        }
        return out;
    }

    double[] getYs(VectorD a, VectorD[] arr) {
        double[] out = new double[arr.length + 1];
        out[0] = a.getY();
        for(int i = 0; i<arr.length; i++) {
            out[i+1] = arr[i].getY();
        }
        return out;
    }

    double[] getZs(VectorD a, VectorD[] arr) {
        double[] out = new double[arr.length + 1];
        out[0] = a.getZ();
        for(int i = 0; i<arr.length; i++) {
            out[i+1] = arr[i].getZ();
        }
        return out;
    }


    public void finish(VectorD eHandle) {
        if (created) return;

        double dist = distance(start, end);
        x = new Polynomial(0, new double[] {start.getX(), normalize(sHandle).multiplied(dist).getX(), 0},
                1, new double[] {end.getX(), normalize(eHandle).multiplied(dist).getX(), 0});
        y = new Polynomial(0, new double[] {start.getY(), normalize(sHandle).multiplied(dist).getY(), 0},
                1, new double[] {end.getY(), normalize(eHandle).multiplied(dist).getY(), 0});
        created = true;
    }




    /*public void paint(Graphics2D g) {
        int dots = 50;
        int[]xs = new int[dots+1];
        int[]ys = new int[dots+1];
        for(double i = 0, t = 0; t<=1; t+=1d/dots, i+=1) {
            xs[(int)i] = (int)x.f(t);
            ys[(int)i] = (int)y.f(t);
        }
        xs[dots] = (int)x.f(1);
        ys[dots] = (int)y.f(1);
        g.drawPolyline(xs, ys, xs.length);
    }

    public VectorD[] dispPoints() {
        int dots = 50;
        VectorD[] out = new VectorD[dots+1];
        for(double i = 0, t = 0; t<=1; t+=1d/dots, i+=1) {
            out[(int)i] = new VectorD(x.f(t), y.f(t));
        }
        out[dots] = new VectorD(x.f(1), y.f(1));
        return out;
    }*/

}
