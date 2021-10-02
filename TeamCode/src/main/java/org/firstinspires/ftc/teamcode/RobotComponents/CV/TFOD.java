package org.firstinspires.ftc.teamcode.RobotComponents.CV;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.NewtonRaphsonSolver;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Common.Util;
import org.firstinspires.ftc.teamcode.Common.VectorD;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.FOCAL_LENGTH;
import static org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants.PRINCIPLE_POINT;

@Disabled
public class TFOD {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "quad";//quad
    private static final String LABEL_SECOND_ELEMENT = "single";//single

    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    private int state = 0;
    private VectorD stackScreenPos = null;

    public int getStackState() {
        return state;
    }

    public int detectStack() {
        if(tfod==null) return -1;

        List<Recognition> recognitions = tfod.getUpdatedRecognitions();
        if (recognitions != null && recognitions.size() > 0) {
            switch(recognitions.get(0).getLabel()) {
                case "single":
                    state = 1;
                    return 1;
                case "quad":
                    state = 2;
                    return 2;
                default:
                    state = 0;
                    return 0;
            }
        }
        return 0;
    }

    public List<VectorD> getPoss() {
        if(tfod==null) return null;
        ArrayList<VectorD> poss = new ArrayList<>();

        List<Recognition> recognitions = tfod.getUpdatedRecognitions();
        if (recognitions != null && recognitions.size() > 0) {
            for(Recognition recognition : recognitions) {
                //VectorD point = new VectorD(recognition.));
            }
        }
        return null;
    }

    public void init(HardwareMap hardwareMap) {
        initVuforia(hardwareMap);
        initTfod(hardwareMap);
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 1.78);
        }
    }

    private void initVuforia(HardwareMap hardwareMap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = RobotConstants.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        RobotLog.d("Bruh " + String.valueOf(vuforia==null));
    }

    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void deActivate() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public VectorD ray(double u, double v) {
        //step 1: transform to pinhole? coords
        double xpp = (u - PRINCIPLE_POINT.getX()) / FOCAL_LENGTH;
        double ypp = (v - PRINCIPLE_POINT.getY()) / FOCAL_LENGTH;

        //step 2: undistort

        double rp = Math.sqrt(xpp*xpp + ypp*ypp);
        PolynomialFunction f = new PolynomialFunction(new double[] {-rp, 1, 0, RobotConstants.K1, 0, RobotConstants.K2, 0, RobotConstants.K3});

        NewtonRaphsonSolver solver = new NewtonRaphsonSolver();
        double solution = solver.solve(100000000, f, 0, 0.656);
        VectorD uD = Util.setMagnitude(new VectorD(xpp, ypp), solution);
        double xp = uD.getX();
        double yp = uD.getY();

        //step 3: ray
        VectorD ray = new VectorD(xp, yp, 1);
        //xp = x/z
        //yp = y/z

        //step 4: orient to robot pose
        return null;
    }

}