package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Common.Spline;
import org.firstinspires.ftc.teamcode.Common.Utilities;
import org.firstinspires.ftc.teamcode.Common.VectorD;

public class SplinePath extends Path {
    private VectorD[] controlPoints;

    public SplinePath(VectorD[] points) {
        super(getSplinedPoints(points));
        controlPoints = points;

    }

    private static VectorD[] getSplinedPoints(VectorD[] points) {
        Spline spline = new Spline(Utilities.extractXs(points), Utilities.extractYs(points));
        //spline.setPointDistance(res);
        VectorD[] splineInterpolated = Utilities.combineXY(spline.getIntX(), spline.getIntY());
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
