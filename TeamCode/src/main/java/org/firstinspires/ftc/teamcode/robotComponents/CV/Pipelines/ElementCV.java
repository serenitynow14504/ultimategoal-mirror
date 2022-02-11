package org.firstinspires.ftc.teamcode.robotComponents.CV.Pipelines;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class ElementCV extends OpenCvPipeline {
    private int finalPos = 1;
    private final int lowerColorHue = 30;
    private final int upperColorHue = 50;

    public static double POS_THRESH_1 = 0.3;
    public static double POS_THRESH_2 = 0.7;


    @Override
    public Mat processFrame(Mat input) {
        Mat convertedInput = new Mat();
        Imgproc.cvtColor(input, convertedInput, Imgproc.COLOR_RGB2HSV);

        List<Mat> HSVChannels = new ArrayList<Mat>(3);
        Core.split(convertedInput, HSVChannels);


        Mat maskHue = HSVChannels.get(0);



        Imgproc.threshold(maskHue, maskHue, lowerColorHue, 0, Imgproc.THRESH_TOZERO);
        Imgproc.threshold(maskHue, maskHue, upperColorHue, 0, Imgproc.THRESH_TOZERO_INV);
        Imgproc.threshold(maskHue, maskHue, 1, 255, Imgproc.THRESH_BINARY);
        //Core.invert(maskHue, maskHue);

        Mat maskSat = HSVChannels.get(1);
        Imgproc.threshold(maskSat, maskSat, 150, 255, Imgproc.THRESH_BINARY);

        Mat maskBright = HSVChannels.get(2);
        Imgproc.threshold(maskBright, maskBright, 30, 255, Imgproc.THRESH_BINARY);

        Mat maskFinal = new Mat();
        Core.bitwise_and(maskHue, maskSat, maskFinal);
        //Core.bitwise_and(maskFinal, maskBright, maskFinal);


        //Mat img = maskFinal.clone();
        //img.convertTo(img, CvType.CV_64FC3);

        int count = 0;
        int avgX = 0, avgY = 0;


        //Core.reduce(maskFinal, maskFinal, 0, Core.REDUCE_SUM, CvType.CV_32S);
//        RobotLog.d("dimensions: " + maskFinal.cols() + ", " + maskFinal.rows());

        for(int i = 0; i<maskFinal.rows(); i++) {
            for(int j = 0; j<maskFinal.cols(); j++) {
                //RobotLog.d("maskfinal.get("+j+", "+i+") null?: " + (maskFinal.get(j, i) == null));
                if(maskFinal.get(j, i) == null) continue;
                if (maskFinal.get(j, i)[0] > 128) {
                    count++;
                    avgX += j;
                    avgY += i;
                }
            }
        }

        Imgproc.line(maskFinal, new Point(POS_THRESH_1*maskFinal.cols(), 0), new Point(POS_THRESH_1*maskFinal.cols(), maskFinal.rows()), new Scalar(128, 128, 128));
        Imgproc.line(maskFinal, new Point(POS_THRESH_2*maskFinal.cols(), 0), new Point(POS_THRESH_2*maskFinal.cols(), maskFinal.rows()), new Scalar(128, 128, 128));

        if(count == 0) return input;

        avgX /= count;
        avgY /= count;

        Imgproc.line(maskFinal, new Point(avgX, 0), new Point(avgX, maskFinal.rows()), new Scalar(255, 255, 128));
        Imgproc.line(maskFinal, new Point(0, avgY), new Point(maskFinal.cols(), avgY), new Scalar(255, 255, 128));

        RobotLog.d("avg: " + avgX + ", " + avgY + "    count: " + count);

        if(avgX < POS_THRESH_1*maskFinal.cols()) {
            finalPos = 0;
        } else if(avgX < POS_THRESH_2*maskFinal.cols()) {
            finalPos = 1;
        } else {
            finalPos = 2;
        }
        return maskFinal;
    }

    public int getFinalPos() {
        return finalPos;
    }
}
