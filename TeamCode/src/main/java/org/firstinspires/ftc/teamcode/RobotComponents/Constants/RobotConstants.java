package org.firstinspires.ftc.teamcode.RobotComponents.Constants;

import com.acmerobotics.dashboard.config.Config;

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

}
