package org.firstinspires.ftc.teamcode.RobotComponents.CV;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Disabled
public class TFOD {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "quad";//quad
    private static final String LABEL_SECOND_ELEMENT = "single";//single

    private static final String VUFORIA_KEY =
            "AWMiTR//////AAABmYXpWn1yJ04SmA7mBd4qGvmN0hVxCBEXwryOr93JrYeogd2OCZFOCT8NQaZbXiGfToGj1u7vNFfE6+RlzaCAxnscNV5ldyA8qvt/ztSlTc7C+vj0ruFzoGU6bft4+hQjQH+KN4z92DpfJUUVhjLfm9gTH9rCNfo23d7wp9nQYSd/MdKcwyHcbBx0iTrWAsbW51BMUVQumnoyc41T/V4WMPVf1OYDdxhj9EhhclrXqPcpgpUJ6v+3+w0ceHQi/VcZGSo4uX7rtLdYSzMZHpKC2ovnYIS5YkGGUWPrIXDb1emSViGU80H6c+V8Zdgn0p1mNm62FIR0BrIlbfDXLjy4ijdhaS4eFdwJadLlariEJi39";

    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public int getStackState(Telemetry telemetry) {
        telemetry.addLine("try");
        if(tfod==null) return -1;

        List<Recognition> recognitions = tfod.getUpdatedRecognitions();
        telemetry.addData("get", recognitions==null);
        if (recognitions != null && recognitions.size() > 0) {
            telemetry.addLine("if");
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
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
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
}