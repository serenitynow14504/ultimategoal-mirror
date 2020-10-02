package org.firstinspires.ftc.teamcode.RobotComponents.PathPlanning;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Common.Spline;
import org.firstinspires.ftc.teamcode.Common.Utilities;

public class SplinePath extends Path {
    private VectorF[] controlPoints;

    public SplinePath(VectorF[] points) {
        super(getSplinedPoints(points));
        controlPoints = points;

    }

    private static VectorF[] getSplinedPoints(VectorF[] points) {
        Spline spline = new Spline(Utilities.extractXs(points), Utilities.extractYs(points));
        //spline.setPointDistance(res);
        VectorF[] splineInterpolated = Utilities.combineXY(spline.getIntX(), spline.getIntY());
        return splineInterpolated;
    }

    public VectorF[] getControlPoints() {
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
