package org.firstinspires.ftc.teamcode.RobotComponents;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.FRONT_RIGHT_ODO_NAME;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.FRONT_RIGHT_ODO_POSE;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.ODO_ENCODER_COUNTS_PER_REV;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.ODO_WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.SIDE_ODO_NAME;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.SIDE_ODO_POSE;


public class TwoWheelOdometry extends OdometryLocalizer {
    private EncoderWheel F, S;
    private Imu imu;

    public TwoWheelOdometry(Robot p) {
        super(p);
        F =  new EncoderWheel(FRONT_RIGHT_ODO_NAME, FRONT_RIGHT_ODO_POSE);
        S =  new EncoderWheel(SIDE_ODO_NAME, SIDE_ODO_POSE);
    }

    protected RealVector calculate() {//According to Ryan's paper
        RealVector angles = new ArrayRealVector(new double[] {
                F.getData()[1]/ODO_ENCODER_COUNTS_PER_REV*2d*PI,
                S.getData()[1]/ODO_ENCODER_COUNTS_PER_REV*2d*PI,
                Math.toRadians(imu.getData()[1])}
                , false);

        //Util.log("angles: " + angles.toString());

        RealMatrix mat = new Array2DRowRealMatrix(new double[][] {
                {sin(F.getPosR()), cos(F.getPosR()),
                        F.getPosY()*sin(F.getPosR())-F.getPosX()*cos(F.getPosR())},
                {sin(S.getPosR()), cos(S.getPosR()),
                        S.getPosY()*sin(S.getPosR())-S.getPosX()*cos(S.getPosR())},
                {0, 0, 1}
        }, false);

        DecompositionSolver solver = new LUDecomposition(mat).getSolver();
        RealVector sol = solver.solve(angles).mapMultiply(ODO_WHEEL_RADIUS);
        //Util.log("change: " + sol);
        //return new VectorD(sol.getEntry(0), sol.getEntry(1), sol.getEntry(2));
        return sol;
    }
    public void setHeading(double degrees) {
        imu.setHeading(degrees);
    }

    public void init(HardwareMap hardwareMap) {
        F.init(hardwareMap);
        S.init(hardwareMap);
        imu = new Imu(hardwareMap.get(BNO055IMU.class, "imu"), parent);
        imu.setAngles();
    }
}
