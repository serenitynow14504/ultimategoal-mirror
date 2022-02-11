package org.firstinspires.ftc.teamcode.robotComponents;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants.FRONT_LEFT_ODO_NAME;
import static org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants.FRONT_LEFT_ODO_POSE;
import static org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants.FRONT_RIGHT_ODO_NAME;
import static org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants.FRONT_RIGHT_ODO_POSE;
import static org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants.ODO_ENCODER_COUNTS_PER_REV;
import static org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants.SIDE_ODO_NAME;
import static org.firstinspires.ftc.teamcode.robotComponents.Constants.RobotConstants.SIDE_ODO_POSE;


public class ThreeWheelOdometry extends OdometryLocalizer {
    public EncoderWheel FL, FR, S;

    private double a, b, c, d, e, f, g, h, i, normalDet;

    private RealMatrix normal, inverted;


    public ThreeWheelOdometry(Robot p) {
        super(p);
        FL =  new EncoderWheel(FRONT_LEFT_ODO_NAME, FRONT_LEFT_ODO_POSE);
        FR =  new EncoderWheel(FRONT_RIGHT_ODO_NAME, FRONT_RIGHT_ODO_POSE);
        S =  new EncoderWheel(SIDE_ODO_NAME, SIDE_ODO_POSE);


        a = cos(FL.getPosR());
        b = sin(FL.getPosR());//0
        c = FL.getPosX() * sin(FL.getPosR()) - FL.getPosY() * cos(FL.getPosR());

        d = cos(FR.getPosR());
        e = sin(FR.getPosR());//0
        f = FR.getPosX() * sin(FR.getPosR()) - FR.getPosY() * cos(FR.getPosR());

        g = cos( S.getPosR());//0
        h = sin( S.getPosR());
        i = S.getPosX() * sin( S.getPosR()) -  S.getPosY() * cos( S.getPosR());


        normalDet = - a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);


        normal = new Array2DRowRealMatrix(new double[][] {
                {a, b, c},
                {d, e, f},
                {g, h, i}
        });


        inverted = new Array2DRowRealMatrix(new double[][] {
                {e*i - f*h, c*h - b*i, b*f - c*e},
                {f*g - d*i, a*i - c*g, c*d - a*f},
                {d*h - e*g, b*g - a*h, a*e - b*d}
        }).scalarMultiply(1d/normalDet);
    }

    protected RealVector calculate() {//According to Ryan's paper
        int[] flData = FL.getData();
        int[] frData = FR.getData();
        int[] sData = S.getData();
//        RobotLog.d("bruh dFL: " + flData[1] + ", dFR: " + frData[1] + ", dS: " + sData[1]);
        RealVector angles = new ArrayRealVector(new double[] {
                flData[1]/ODO_ENCODER_COUNTS_PER_REV*2d*PI,
                frData[1]/ODO_ENCODER_COUNTS_PER_REV*2d*PI,
                sData[1]/ODO_ENCODER_COUNTS_PER_REV*2d*PI}
                , false);
//        RobotLog.d("bruh angles: " + angles.toString());
//        RobotLog.d("bruh matrix: " + inverted.toString());
//        RobotLog.d("bruh normal matrix: " + normal.toString());
        //Util.log("angles: " + angles.toString());


        DecompositionSolver solver = new LUDecomposition(normal).getSolver();
        RealVector sol = solver.solve(angles);
        //Util.log("change: " + sol);
        //return new VectorD(sol.getEntry(0), sol.getEntry(1), sol.getEntry(2));

        //return mat.operate(angles).mapMultiplyToSelf(ODO_WHEEL_RADIUS/4d);
//        return inverted.operate(angles).mapMultiplyToSelf(ODO_WHEEL_RADIUS);

        return sol;
    }
    public void setHeading(double degrees) {
        //
    }

    public void init(HardwareMap hardwareMap) {
        FL.init(hardwareMap);
        FR.init(hardwareMap);
        S.init(hardwareMap);
    }
}
