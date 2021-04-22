package org.firstinspires.ftc.teamcode.RobotComponents;

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
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.FRONT_LEFT_ODO_NAME;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.FRONT_LEFT_ODO_POSE;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.FRONT_RIGHT_ODO_NAME;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.FRONT_RIGHT_ODO_POSE;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.ODO_ENCODER_COUNTS_PER_REV;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.ODO_WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.SIDE_ODO_NAME;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.SIDE_ODO_POSE;


public class ThreeWheelOdometry extends OdometryLocalizer {
    public EncoderWheel FL, FR, S;

    public ThreeWheelOdometry(Robot p) {
        super(p);
        FL =  new EncoderWheel(FRONT_LEFT_ODO_NAME, FRONT_LEFT_ODO_POSE);
        FR =  new EncoderWheel(FRONT_RIGHT_ODO_NAME, FRONT_RIGHT_ODO_POSE);
        S =  new EncoderWheel(SIDE_ODO_NAME, SIDE_ODO_POSE);
    }

    protected RealVector calculate() {//According to Ryan's paper
        RealVector angles = new ArrayRealVector(new double[] {
                FL.getData()[1]/ODO_ENCODER_COUNTS_PER_REV*2d*PI,
                FR.getData()[1]/ODO_ENCODER_COUNTS_PER_REV*2d*PI,
                S.getData()[1]/ODO_ENCODER_COUNTS_PER_REV*2d*PI}
                , false);
        //Util.log("angles: " + angles.toString());

        RealMatrix mat = new Array2DRowRealMatrix(new double[][] {
                {sin(FL.getPosR()), cos(FL.getPosR()),
                        FL.getPosY()*sin(FL.getPosR())-FL.getPosX()*cos(FL.getPosR())},
                {sin(FR.getPosR()), cos(FR.getPosR()),
                        FR.getPosY()*sin(FR.getPosR())-FR.getPosX()*cos(FR.getPosR())},
                {sin(S.getPosR()), cos(S.getPosR()),
                        S.getPosY()*sin(S.getPosR())-S.getPosX()*cos(S.getPosR())}
        });

        DecompositionSolver solver = new LUDecomposition(mat).getSolver();
        RealVector sol = solver.solve(angles).mapMultiply(ODO_WHEEL_RADIUS);
        //Util.log("change: " + sol);
        //return new VectorD(sol.getEntry(0), sol.getEntry(1), sol.getEntry(2));
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
