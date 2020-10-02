package org.firstinspires.ftc.teamcode.RobotComponents.CV.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class skyStoneCV extends OpenCvPipeline {
    private int finalPos = 2;

    private LinearOpMode parent;
    private Robot robot;
    public skyStoneCV(LinearOpMode p, Robot r) {
        parent = p;
        robot = r;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        List<Mat> HSVChannels = new ArrayList<Mat>(3);
        Core.split(input, HSVChannels);


        Mat maskHue = HSVChannels.get(0);
        Imgproc.threshold(maskHue, maskHue, 10, 0, Imgproc.THRESH_TOZERO);
        Imgproc.threshold(maskHue, maskHue, 40, 0, Imgproc.THRESH_TOZERO_INV);
        Imgproc.threshold(maskHue, maskHue, 1, 255, Imgproc.THRESH_BINARY);


        Mat maskSat = HSVChannels.get(1);
        Imgproc.threshold(maskSat, maskSat, 150, 255, Imgproc.THRESH_BINARY);

        Mat maskFinal = new Mat();
        Core.bitwise_and(maskHue, maskSat, maskFinal);


        //Mat img = maskFinal.clone();
        //img.convertTo(img, CvType.CV_64FC3);

        int[] columns = new int[maskFinal.cols()];

        //Core.reduce(maskFinal, maskFinal, 0, Core.REDUCE_SUM, CvType.CV_32S);



        for(int i = 0; i<maskFinal.cols(); i++) {
            columns[i] = 0;

            for(int j = 3*maskFinal.rows()/5; j<maskFinal.rows(); j++) {
                double[] data = maskFinal.get(j, i);

                Imgproc.line(maskFinal, new Point(0, 3*maskFinal.rows()/5), new Point(maskFinal.cols(), 3*maskFinal.rows()/5), new Scalar(128, 0, 0));


                if(data != null) {
                    if (data[0] > 128) {
                        columns[i]++;
                    }
                } else {
                    Imgproc.line( maskFinal, new Point(i, j), new Point(i, j), new Scalar(128, 0, 0));
                }
            }



            if(columns[i] < 25) {
                //Imgproc.line(maskFinal, new Point((double)i, 0.0), new Point((double)i, (double)maskFinal.rows()), new Scalar(128, 0, 0), 1);
                columns[i] = 1;
            } else {
                columns[i] = 0;
            }
                /*if(i == 0) {
                    telemetry.addData("column 0 sum", columns[i]);
                    telemetry.update();
                }
                if(i==1) {
                    Imgproc.line(maskFinal, new Point((double)i, 0.0), new Point((double)i, (double)maskFinal.rows()), new Scalar(128, 0, 0), 1);
                }*/
        }


        int[] skyStonePositions = new int[2];
        skyStonePositions[0] = -1;
        skyStonePositions[1] = -1;


        boolean inABlock = false;

        int sum = 0;
        int count = 0;
        int stonesFound = 0;


        if(robot.getAlliance() == RobotConstants.ALLIANCES.RED) {
            for(int i = 2; i<columns.length-2; i++) {
                if(columns[i]==1 && columns[i-2]==1 && columns[i-1]==1 && columns[i+2]==1 && columns[i+1]==1) {
                    count++;
                    sum += i;

                    if(!inABlock) {inABlock = true;}
                } else {
                    if(inABlock) {
                        if(count > 3 && stonesFound < 2) {  //5
                            skyStonePositions[stonesFound] = sum / count;
                            stonesFound++;

                            Imgproc.line(maskFinal, new Point((double)i, 0.0), new Point((double)i, (double)maskFinal.rows()), new Scalar(128, 0, 0), 1);
                        }
                        sum = 0;
                        count = 0;

                        inABlock = false;
                    }
                }
            }


            double factor = (double) maskFinal.cols() / 5.0;
            finalPos = ((int)(skyStonePositions[0] / factor) + 1) % 3;


        } else if(robot.getAlliance() == RobotConstants.ALLIANCES.BLUE){
            Imgproc.line(maskFinal, new Point(85, 0.0), new Point(85, (double)maskFinal.rows()), new Scalar(128, 0, 0), 2);
            for(int i = 85; i<columns.length-2; i++) {
                if(columns[i]==1 && columns[i-2]==1 && columns[i-1]==1 && columns[i+2]==1 && columns[i+1]==1) {
                    count++;
                    sum += i;

                    if(!inABlock) {inABlock = true;}
                } else {
                    if(inABlock) {
                        if(count > 3 && stonesFound < 2) {  //5
                            skyStonePositions[stonesFound] = sum / count - 85;
                            stonesFound++;

                            Imgproc.line(maskFinal, new Point((double)i, 0.0), new Point((double)i, (double)maskFinal.rows()), new Scalar(128, 0, 0), 1);
                        }
                        sum = 0;
                        count = 0;

                        inABlock = false;
                    }
                }
            }


            double factor = (double) maskFinal.cols() / 5.0;
            //TelemetryPacket packet = new TelemetryPacket();

            //packet.addLine("column" + skyStonePositions[0]);
            //packet.addLine("factor" + factor);


            finalPos = (-((int)((skyStonePositions[0] + 10) / factor)) + 8) % 3;
            //packet.addLine("skystone " + finalPos);

            //robot.dashboard.sendTelemetryPacket(packet);
        }




        return maskFinal;
    }

    public int getFinalPos() {
        return finalPos;
    }
}
