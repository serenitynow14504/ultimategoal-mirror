package org.firstinspires.ftc.teamcode.robotComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.VectorD;

public class EncoderWheel {
    DcMotor encoder;
    private final VectorD pose;
    private int sign = 1;
    private final String name;

    private int prevPos = Integer.MAX_VALUE;

    public EncoderWheel(String name, VectorD pose) {
        this.name = name;
        this.pose = pose;
        //if(flipped) sign = -1;
    }

    public double getPosX() {
        return pose.getX();
    }
    public double getPosY() {
        return pose.getY();
    }
    public double getPosR() {
        return pose.getZ();
    }

    public EncoderWheel(String name, double x, double y, double r) {
        this(name, new VectorD(x, y, r));
        //if(flipped) sign = -1;
    }

    public void init(HardwareMap hardwareMap) {
        encoder = hardwareMap.get(DcMotor.class, name);
        //if(encoder.getDirection().equals(DcMotorSimple.Direction.REVERSE)) sign = -1;
    }

    public int[] getData() { //RETURNS: [Position, Difference]
        int currPos = encoder.getCurrentPosition()*sign;
        int diff = currPos - prevPos;
        if(prevPos == Integer.MAX_VALUE) {
            diff = 0;
        }
        prevPos = currPos;
        return new int[]{currPos, diff};
    }
}
