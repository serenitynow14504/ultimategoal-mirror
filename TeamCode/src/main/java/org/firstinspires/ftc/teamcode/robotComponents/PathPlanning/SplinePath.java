package org.firstinspires.ftc.teamcode.robotComponents.PathPlanning;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.common.Spline;
import org.firstinspires.ftc.teamcode.common.Util;
import org.firstinspires.ftc.teamcode.common.VectorD;

public class SplinePath extends Path {
    private VectorD[] controlPoints;

    public SplinePath(VectorD[] points) {
        super(getSplinedPoints(points));
        controlPoints = points;
        //Utilities.log("splined path created");
    }

    private static VectorD[] getSplinedPoints(VectorD[] points) {
        Spline spline = new Spline(Util.extractXs(points), Util.extractYs(points));
        //spline.setPointDistance(res);
        VectorD[] splineInterpolated = Util.combineXY(spline.getIntX(), spline.getIntY());
        splineInterpolated[splineInterpolated.length-1] = points[points.length-1];
        return splineInterpolated;
    }

    public VectorD[] getControlPoints() {
        return controlPoints;
    }

    @Override
    public void show(TelemetryPacket packet, String color) {
        packet.fieldOverlay().setStrokeWidth(1).setStroke(color).strokePolyline(getXs(), getYs());

        for(int i = 0; i < controlPoints.length; i++) {
            packet.fieldOverlay().setFill(color).setStroke(color).fillCircle(controlPoints[i].get(0), controlPoints[i].get(1), 2);
        }
    }
}
