package org.firstinspires.ftc.teamcode.RobotComponents.Constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Common.VectorD;

@Config
public abstract class RobotConstants {
    //odometry
    public static final double ODO_ENCODER_COUNTS_PER_REV = 8192;
    public static final double ODO_WHEEL_CIRC = 2 * Math.PI;
    public static final double INCHES_TO_COUNTS_CONVERSION = ODO_ENCODER_COUNTS_PER_REV/ODO_WHEEL_CIRC;
    public static final double COUNTS_TO_INCHES_CONVERSION = ODO_WHEEL_CIRC/ODO_ENCODER_COUNTS_PER_REV;

    //translate
    public static final double rampDistThresh = 10;
    public static final double rampDistFrac = 0.3;
    public static final double minPower = 0.2;

    public static final double TXP = 0.00015;
    public static final double TXI = 0.0000005;//0.0000004
    public static final double TXD = 0.0005;
    public static final double TYP = 0.0001;
    public static final double TYI = 0.00000025;//0.00000015
    public static final double TYD = 0.0005;
    public static final double TRP = 0.015;
    public static final double TRI = 0;
    public static final double TRD = 0.009;

    public static final double RP = 0.006;
    public static final double RI = 0.00006;
    public static final double RD = 0.008;//0.0014

    public static final double TXTolerance = 6.0;
    public static final double TYTolerance = 6.0;


    public static final double lookAheadConstant = 18.0;


    public static final double COUNTS_PER_MOTOR_REV = 2240;
    public static final double DRIVE_GEAR_REDUCTION = 2.0;
    public static final double WHEEL_DIAMETER_INCHES = 100 / 25.4;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public static final double CENTER_TO_WHEEL_DIST = 6.25;

    public static final float width = 17.5f;
    public static final float length = 18;

    public enum ALLIANCES {
        RED, BLUE, SOLO
    }

    public static double SHOOTING_OFFSET_ANGLE = 20;

    public static final String VUFORIA_KEY =
            "AWMiTR//////AAABmYXpWn1yJ04SmA7mBd4qGvmN0hVxCBEXwryOr93JrYeogd2OCZFOCT8NQaZbXiGfToGj1u7vNFfE6+RlzaCAxnscNV5ldyA8qvt/ztSlTc7C+vj0ruFzoGU6bft4+hQjQH+KN4z92DpfJUUVhjLfm9gTH9rCNfo23d7wp9nQYSd/MdKcwyHcbBx0iTrWAsbW51BMUVQumnoyc41T/V4WMPVf1OYDdxhj9EhhclrXqPcpgpUJ6v+3+w0ceHQi/VcZGSo4uX7rtLdYSzMZHpKC2ovnYIS5YkGGUWPrIXDb1emSViGU80H6c+V8Zdgn0p1mNm62FIR0BrIlbfDXLjy4ijdhaS4eFdwJadLlariEJi39";


    public static final VectorD GRABBER_POS = new VectorD(-12, 4);//MEASURE PLS
    public static final VectorD INTAKE_POS = new VectorD(0, 8);//MEASURE PLS
    public static final VectorD CAMERA_POS = new VectorD(0, 9, 6.5);

    //==============CAMERA PARAMS==================\\

    public static final int SCREEN_WIDTH = 640;
    public static final int SCREEN_HEIGHT = 480;
    public static final double FOCAL_LENGTH = 822.317;
    public static final VectorD PRINCIPLE_POINT = new VectorD(319.495, 242.502);
    public static final double K1 = -0.0449369;
    public static final double K2 = 1.17277;
    public static final double P1 = 0;
    public static final double P2 = 0;
    public static final double K3 = -3.63244;
    public static final double K4 = 0;
    public static final double K5 = 0;
    public static final double K6 = 0;

}