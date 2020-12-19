package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning.Paths;

import org.firstinspires.ftc.teamcode.Common.Polynomial;
import org.firstinspires.ftc.teamcode.Common.VectorD;
import static  org.firstinspires.ftc.teamcode.Common.Utilities.*;


class PolynomialSegment {
    VectorD start, end, sHandle, eHandle;
    boolean created = false;
    Polynomial x, y;
    public PolynomialSegment(VectorD start, VectorD sHandle, VectorD end, VectorD eHandle) {
        this.start = start;
        this.end = end;
        this.sHandle = sHandle;
        this.eHandle = eHandle;

        double dist = distance(start, end);
        x = new Polynomial(new VectorD(0, start.getX()), normalize(sHandle).multiplied(dist).getX(), 0,
                new VectorD(1, end.getX()), normalize(eHandle).multiplied(dist).getX(), 0);
        y = new Polynomial(new VectorD(0, start.getY()), normalize(sHandle).multiplied(dist).getY(), 0,
                new VectorD(1, end.getY()), normalize(eHandle).multiplied(dist).getY(), 0);
        created = true;
    }

    public PolynomialSegment(VectorD start, VectorD sHandle, VectorD end) {
        this.start = start;
        this.end = end;
        this.sHandle = sHandle;
    }

    public void finish(VectorD eHandle) {
        if(created) return;

        double dist = distance(start, end);
        x = new Polynomial(new VectorD(0, start.getX()), normalize(sHandle).multiplied(dist).getX(), 0,
                new VectorD(1, end.getX()), normalize(eHandle).multiplied(dist).getX(), 0);
        y = new Polynomial(new VectorD(0, start.getY()), normalize(sHandle).multiplied(dist).getY(), 0,
                new VectorD(1, end.getY()), normalize(eHandle).multiplied(dist).getY(), 0);
        created = true;
    }
}
