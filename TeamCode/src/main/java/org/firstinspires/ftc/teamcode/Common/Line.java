package org.firstinspires.ftc.teamcode.Common;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Line {
    private double slope, yIntercept;
    private double paramXSlope, paramXIntercept, paramYSlope, paramYIntercept, d;
    private VectorF a, b;
    private boolean vertical = false;
    private int pathPos;

    public Line(VectorF a, VectorF b, int pos) {
        this.a = new VectorF(a.get(0), a.get(1));
        this.b = new VectorF(b.get(0), b.get(1));
        pathPos = pos;

        if(a.get(0) != b.get(0)) {
            slope = (a.get(1) - b.get(1)) / (a.get(0) - b.get(0));
            yIntercept = a.get(1) - slope * a.get(0);
        } else {
            vertical = true;
            slope = Double.POSITIVE_INFINITY;
            yIntercept = Double.NaN;
        }

        d = Utilities.distance(a, b);
        paramXIntercept = a.get(0);
        paramXSlope = (b.get(0) - a.get(0)) / d;

        paramYIntercept = a.get(1);
        paramYSlope = (b.get(1) - a.get(1)) / d;
    }

    private Line(double slope, double yIntercept) {
        if(Double.isInfinite(slope)) {
            vertical = true;
            this.yIntercept = Double.NaN;

        } else {
            this.yIntercept = yIntercept;
            a = new VectorF(0, (float)yIntercept);
            b = new VectorF(1, (float)(slope + yIntercept));
        }
        this.slope = slope;
        d = Utilities.distance(a, b);

    }

    public Line(VectorF point, double slop) {
        slope = slop;
        if(Double.isInfinite(slope)) {
            vertical = true;
            this.yIntercept = Double.NaN;

            a = new VectorF(point.get(0), 0);
            b = new VectorF(point.get(0), 1);
        } else {
            yIntercept = point.get(1) - slope * point.get(0);

            a = new VectorF(point.get(0), point.get(1)); //a = new VectorF(point.get(0), point
            // .get(2));
            b = new VectorF((float)3.14, (float)(slope*3.14 + yIntercept));
        }
    }

    private double getPerpSlope() {
        if(slope == 0) {
            return Double.POSITIVE_INFINITY;
        }
        return -1/slope;
    }

    public VectorF getPointA() {
        return Utilities.duplicate(a);
    }
    public VectorF getPointB() {
        return Utilities.duplicate(b);
    }

    double getSlope() {return slope;}
    double getyIntercept() {return yIntercept;}
    boolean isVertical() {return vertical;}
    public double getLength() {return d;}
    public int getPathPos() {return pathPos;}

    public Line perpLine(VectorF point) {
        return new Line(point, getPerpSlope());
    }

    public VectorF intersect(Line l) {
        if(slope == l.getSlope()) {
            if(yIntercept == l.getyIntercept()) {
                //throw new Exception();
                return null;
            } else {
                //throw new Ex
                return null;
            }
        }


        double x, y;
        if(vertical) {
            x = a.get(0);
            y = l.getSlope()*x + l.yIntercept;
        } else if(l.isVertical()) {
            x = l.a.get(0);
            y = slope * x + yIntercept;
        } else {
            x = (l.getyIntercept() - yIntercept) / (slope - l.getSlope());
            y = slope * x + yIntercept;
        }

         return  new VectorF((float)x, (float)y);
    }

    public Pair<VectorF, Double> getClosestPoint(VectorF point) {
        VectorF intersection = intersect(perpLine(point));
        VectorF closestPoint;
        if(!vertical) {
            if (a.get(0) < b.get(0)) {
                if(intersection.get(0) < a.get(0)) {
                    closestPoint = a;
                } else if (intersection.get(0) > b.get(0)) {
                    closestPoint = b;
                } else {
                    closestPoint = intersection;
                }
            } else {
                if(intersection.get(0) < b.get(0)) {
                    closestPoint = b;
                } else if (intersection.get(0) > a.get(0)) {
                    closestPoint = a;
                } else {
                    closestPoint = intersection;
                }
            }
        } else {
            if (a.get(1) < b.get(1)) {
                if(intersection.get(1) < a.get(1)) {
                    closestPoint = a;
                } else if (intersection.get(1) > b.get(1)) {
                    closestPoint = b;
                } else {
                    closestPoint = intersection;
                }
            } else {
                if(intersection.get(1) < b.get(1)) {
                    closestPoint = b;
                } else if (intersection.get(1) > a.get(1)) {
                    closestPoint = a;
                } else {
                    closestPoint = intersection;
                }
            }
        }

        return new Pair<>(closestPoint, Utilities.distance(point, closestPoint));
    }

    public VectorF closestVector(VectorF point) {
        /*VectorF intersection = intersect(perpLine(point));
        VectorF closestPoint;
        if(!vertical) {
            if (a.get(0) < b.get(0)) {
                if(intersection.get(0) < a.get(0)) {
                    closestPoint = a;
                } else if (intersection.get(0) > b.get(0)) {
                    closestPoint = b;
                } else {
                    closestPoint = intersection;
                }
            } else {
                if(intersection.get(0) < b.get(0)) {
                    closestPoint = b;
                } else if (intersection.get(0) > a.get(0)) {
                    closestPoint = a;
                } else {
                    closestPoint = intersection;
                }
            }
        } else {
            if (a.get(1) < b.get(1)) {
                if(intersection.get(1) < a.get(1)) {
                    closestPoint = a;
                } else if (intersection.get(1) > b.get(1)) {
                    closestPoint = b;
                } else {
                    closestPoint = intersection;
                }
            } else {
                if(intersection.get(1) < b.get(1)) {
                    closestPoint = b;
                } else if (intersection.get(1) > a.get(1)) {
                    closestPoint = a;
                } else {
                    closestPoint = intersection;
                }
            }
        }

        return closestPoint.subtracted(point);*/

        //return getClosestPoint(point).first.subtracted(point);
        return point.subtracted(getClosestPoint(point).first);
    }

    public VectorF closestVector(Line line) {
        VectorF vector;

        if(slope == line.getSlope()) {
            Line perp = line.perpLine(line.getPointA());
            vector = line.intersect(perp).subtracted(intersect(perp));
        } else {
            VectorF p1 = line.getClosestPoint(getPointA()).first.subtracted(a);
            VectorF p2 = line.getClosestPoint(getPointB()).first.subtracted(b);

            VectorF p3 = line.getPointA().subtracted(getClosestPoint(line.getPointA()).first);
            VectorF p4 = line.getPointB().subtracted(getClosestPoint(line.getPointB()).first);

            VectorF[] ps = new VectorF[] {
                    p1, p2, p3, p4
            };

            vector = Utilities.minLengthVector(ps);
        }

        return vector;
    }


     boolean facing(VectorF point) {
        if(slope != 0) {
            Line aPerp = perpLine(a);
            Line bPerp = perpLine(b);

            if((point.get(1) >= point.get(0) * aPerp.getSlope() + aPerp.getyIntercept() &&
                        point.get(1) <= point.get(0) * bPerp.getSlope() + bPerp.getyIntercept()) ||
                    (point.get(1) <= point.get(0) * aPerp.getSlope() + aPerp.getyIntercept() &&
                        point.get(1) >= point.get(0) * bPerp.getSlope() + bPerp.getyIntercept())) {
                return true;
            }
        } else {
            if((point.get(0) >= a.get(0) && point.get(0) <= b.get(0)) ||
                    (point.get(0) <= a.get(0) && point.get(0) >= b.get(0))) {
                return true;
            }

        }

        return false;
    }

    public int calculateLeftRight(VectorF point) {
        VectorF clippedPoint = Utilities.clipToXY(point);
        VectorF toPath = intersect(perpLine(clippedPoint)).subtracted(clippedPoint);
        VectorF toRight = new VectorF(this.toVector().get(1), -this.toVector().get(0));
        float dot = toRight.dotProduct(toPath);
        return (int)Math.signum(dot);
    }


    public VectorF getParamatrizedPoint(double t) {
        return new VectorF((float)(t*paramXSlope + paramXIntercept), (float)(t*paramYSlope + paramYIntercept));
    }

    public double getParameter(VectorF point) {
        return Utilities.distance(a, point);
    }

    public double yFromX(double x) {
        return slope * x + yIntercept;
    }

    public VectorF toVector() {
        return getPointB().subtracted(getPointA());
    }

}
