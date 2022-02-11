package org.firstinspires.ftc.teamcode.common;

import android.util.Pair;



public class Line {
    private double slope, yIntercept;
    private double paramXSlope, paramXIntercept, paramYSlope, paramYIntercept, d;
    private VectorD a, b;
    private boolean vertical = false;
    private int pathPos;

    public Line(VectorD a, VectorD b, int pos) {
        this.a = new VectorD(a.getX(), a.getY());
        this.b = new VectorD(b.getX(), b.getY());
        pathPos = pos;

        if(a.getX() != b.getX()) {
            slope = (a.getY() - b.getY()) / (a.getX() - b.getX());
            yIntercept = a.getY() - slope * a.getX();
        } else {
            vertical = true;
            slope = Double.POSITIVE_INFINITY;
            yIntercept = Double.NaN;
        }

        d = Util.distance(a, b);
        paramXIntercept = a.getX();
        paramXSlope = (b.getX() - a.getX()) / d;

        paramYIntercept = a.getY();
        paramYSlope = (b.getY() - a.getY()) / d;
    }

    private Line(double slope, double yIntercept) {
        if(Double.isInfinite(slope)) {
            vertical = true;
            this.yIntercept = Double.NaN;

        } else {
            this.yIntercept = yIntercept;
            a = new VectorD(0, (float)yIntercept);
            b = new VectorD(1, (float)(slope + yIntercept));
        }
        this.slope = slope;
        d = Util.distance(a, b);

    }

    public Line(VectorD point, double slop) {
        slope = slop;
        if(Double.isInfinite(slope)) {
            vertical = true;
            this.yIntercept = Double.NaN;

            a = new VectorD(point.getX(), 0);
            b = new VectorD(point.getX(), 1);
        } else {
            yIntercept = point.getY() - slope * point.getX();

            a = new VectorD(point.getX(), point.getY()); //a = new VectorD(point.getX(), point
            // .get(2));
            b = new VectorD((float)3.14, (float)(slope*3.14 + yIntercept));
        }
    }

    public Line(VectorD a, VectorD b) {
        this(a, (b.getY()-a.getY())/(b.getX()-a.getX()));
    }

    private double getPerpSlope() {
        if(slope == 0) {
            return Double.POSITIVE_INFINITY;
        }
        return -1/slope;
    }

    public VectorD getPointA() {
        return Util.duplicate(a);
    }
    public VectorD getPointB() {
        return Util.duplicate(b);
    }

    double getSlope() {return slope;}
    double getyIntercept() {return yIntercept;}
    boolean isVertical() {return vertical;}
    public double getLength() {return d;}
    public int getPathPos() {return pathPos;}

    public Line perpLine(VectorD point) {
        return new Line(point, getPerpSlope());
    }

    public VectorD intersect(Line l) {
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
            x = a.getX();
            y = l.getSlope()*x + l.yIntercept;
        } else if(l.isVertical()) {
            x = l.a.getX();
            y = slope * x + yIntercept;
        } else {
            x = (l.getyIntercept() - yIntercept) / (slope - l.getSlope());
            y = slope * x + yIntercept;
        }

         return  new VectorD((float)x, (float)y);
    }

    public Pair<VectorD, Double> getClosestPoint(VectorD point) {
        VectorD intersection = intersect(perpLine(point));
        VectorD closestPoint;
        if(!vertical) {
            if (a.getX() < b.getX()) {
                if(intersection.getX() < a.getX()) {
                    closestPoint = a;
                } else if (intersection.getX() > b.getX()) {
                    closestPoint = b;
                } else {
                    closestPoint = intersection;
                }
            } else {
                if(intersection.getX() < b.getX()) {
                    closestPoint = b;
                } else if (intersection.getX() > a.getX()) {
                    closestPoint = a;
                } else {
                    closestPoint = intersection;
                }
            }
        } else {
            if (a.getY() < b.getY()) {
                if(intersection.getY() < a.getY()) {
                    closestPoint = a;
                } else if (intersection.getY() > b.getY()) {
                    closestPoint = b;
                } else {
                    closestPoint = intersection;
                }
            } else {
                if(intersection.getY() < b.getY()) {
                    closestPoint = b;
                } else if (intersection.getY() > a.getY()) {
                    closestPoint = a;
                } else {
                    closestPoint = intersection;
                }
            }
        }

        return new Pair<>(closestPoint, Util.distance(point, closestPoint));
    }

    public VectorD closestVector(VectorD point) {
        /*VectorD intersection = intersect(perpLine(point));
        VectorD closestPoint;
        if(!vertical) {
            if (a.getX() < b.getX()) {
                if(intersection.getX() < a.getX()) {
                    closestPoint = a;
                } else if (intersection.getX() > b.getX()) {
                    closestPoint = b;
                } else {
                    closestPoint = intersection;
                }
            } else {
                if(intersection.getX() < b.getX()) {
                    closestPoint = b;
                } else if (intersection.getX() > a.getX()) {
                    closestPoint = a;
                } else {
                    closestPoint = intersection;
                }
            }
        } else {
            if (a.getY() < b.getY()) {
                if(intersection.getY() < a.getY()) {
                    closestPoint = a;
                } else if (intersection.getY() > b.getY()) {
                    closestPoint = b;
                } else {
                    closestPoint = intersection;
                }
            } else {
                if(intersection.getY() < b.getY()) {
                    closestPoint = b;
                } else if (intersection.getY() > a.getY()) {
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

    public VectorD closestVector(Line line) {
        VectorD vector;

        if(slope == line.getSlope()) {
            Line perp = line.perpLine(line.getPointA());
            vector = line.intersect(perp).subtracted(intersect(perp));
        } else {
            VectorD p1 = line.getClosestPoint(getPointA()).first.subtracted(a);
            VectorD p2 = line.getClosestPoint(getPointB()).first.subtracted(b);

            VectorD p3 = line.getPointA().subtracted(getClosestPoint(line.getPointA()).first);
            VectorD p4 = line.getPointB().subtracted(getClosestPoint(line.getPointB()).first);

            VectorD[] ps = new VectorD[] {
                    p1, p2, p3, p4
            };

            vector = Util.minLengthVector(ps);
        }

        return vector;
    }


     boolean facing(VectorD point) {
        if(slope != 0) {
            Line aPerp = perpLine(a);
            Line bPerp = perpLine(b);

            if((point.getY() >= point.getX() * aPerp.getSlope() + aPerp.getyIntercept() &&
                        point.getY() <= point.getX() * bPerp.getSlope() + bPerp.getyIntercept()) ||
                    (point.getY() <= point.getX() * aPerp.getSlope() + aPerp.getyIntercept() &&
                        point.getY() >= point.getX() * bPerp.getSlope() + bPerp.getyIntercept())) {
                return true;
            }
        } else {
            if((point.getX() >= a.getX() && point.getX() <= b.getX()) ||
                    (point.getX() <= a.getX() && point.getX() >= b.getX())) {
                return true;
            }

        }

        return false;
    }

    public int calculateLeftRight(VectorD point) {
        VectorD clippedPoint = Util.clipToXY(point);
        VectorD toPath = intersect(perpLine(clippedPoint)).subtracted(clippedPoint);
        VectorD toRight = new VectorD(this.toVector().getY(), -this.toVector().getX());
        float dot = toRight.dotProduct(toPath);
        return (int)Math.signum(dot);
    }


    public VectorD getParamatrizedPoint(double t) {
        return new VectorD((float)(t*paramXSlope + paramXIntercept), (float)(t*paramYSlope + paramYIntercept));
    }

    public double getParameter(VectorD point) {
        return Util.distance(a, point);
    }

    public double yFromX(double x) {
        return slope * x + yIntercept;
    }

    public VectorD toVector() {
        return getPointB().subtracted(getPointA());
    }

}
