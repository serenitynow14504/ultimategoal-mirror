package org.firstinspires.ftc.teamcode.RobotComponents.CV.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotComponents.Robot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RingCV extends OpenCvPipeline {
    private int finalPos = 2;
    private final int lowerColorHue = 10;
    private final int upperColorHue = 40;
    private final double sampleCol = 0.5;

    private LinearOpMode parent;
    private Robot robot;
    public RingCV(LinearOpMode o, Robot r) {
        parent = o;
        robot = r;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        List<Mat> HSVChannels = new ArrayList<Mat>(3);
        Core.split(input, HSVChannels);


        Mat maskHue = HSVChannels.get(0);
        Imgproc.threshold(maskHue, maskHue, lowerColorHue, 0, Imgproc.THRESH_TOZERO);
        Imgproc.threshold(maskHue, maskHue, upperColorHue, 0, Imgproc.THRESH_TOZERO_INV);
        Imgproc.threshold(maskHue, maskHue, 1, 255, Imgproc.THRESH_BINARY);


        Mat maskSat = HSVChannels.get(1);
        Imgproc.threshold(maskSat, maskSat, 150, 255, Imgproc.THRESH_BINARY);

        Mat maskFinal = new Mat();
        Core.bitwise_and(maskHue, maskSat, maskFinal);


        //Mat img = maskFinal.clone();
        //img.convertTo(img, CvType.CV_64FC3);

        int count = 0;

        //Core.reduce(maskFinal, maskFinal, 0, Core.REDUCE_SUM, CvType.CV_32S);


        for(int i = 0; i<maskFinal.rows(); i++) {
            count += (maskFinal.get((int)(sampleCol*maskFinal.cols()), i)[0] > 128) ? 1 : 0;
        }

        if(count < 10) finalPos = 0;
        else if(count < 25) finalPos = 1;
        else finalPos = 2;

        return maskFinal;
    }

    public int getFinalPos() {
        return finalPos;
    }
}
