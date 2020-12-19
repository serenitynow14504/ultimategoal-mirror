package org.firstinspires.ftc.teamcode.RobotComponents.CV;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RobotComponents.Constants.RobotConstants;

import java.util.List;

@Disabled
public class TFOD {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "quad";//quad
    private static final String LABEL_SECOND_ELEMENT = "single";//single

    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public int getStackState() {
        if(tfod==null) return -1;

        List<Recognition> recognitions = tfod.getUpdatedRecognitions();
        if (recognitions != null && recognitions.size() > 0) {
            switch(recognitions.get(0).getLabel()) {
                case "single":
                    return 1;
                case "quad":
                    return 2;
                default:
                    return 0;
            }
        }
        return 0;
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
}