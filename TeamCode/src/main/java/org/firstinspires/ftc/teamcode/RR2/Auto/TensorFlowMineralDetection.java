package org.firstinspires.ftc.teamcode.RR2.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;

import java.util.List;

public class TensorFlowMineralDetection {
    public enum Location{
        CENTER,LEFT,RIGHT,UNKNOWN;
    }

    public HardwareMap hardwareMap;
    public TensorFlowMineralDetection(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public Location location;
    public Telemetry telemetry;
    public LinearOpMode linearOpMode;

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    public static final String VUFORIA_KEY = "AQOQmUj/////AAABmXCrndWask2hud2XzSvIrY52Fjls7yhcRKcQMVgooTqhv7MCYikM1d8E8z5LHk0DHRlwk+Qwvhl1k+p6NIRSQ4dHtbxYxpD0nO9pEhtlY/ABsjRyS+QrC3xqImfkY+IL6zNXtySZjozAhoNmP2sIx7JBN6hcpdabhrywRplCBfOh2uUI3FLMD544Lo6BIHST42mTExPyUIRmCLf4JHEavNAa3cC19X8IRzfm7cWlKLbAJCzNls2Tkp/wkUAdRSBdDQ4156qBsiIC5XoFVhdz+M7o62+MqKlDa6bm+VcYvex8gkAwRaoiOYGyzIdVLvbnbAAdAdjApeDozBzukAjsjF3tPltcVYyIJRM5mRWeNJGj";

    public VuforiaLocalizer vuforia;

    public TFObjectDetector tfod;

    public WebcamName webcamName;

    boolean isGoldInFrame = false;

    public Location runObjectDetection() throws VuforiaException{

        if (tfod != null) {
            tfod.activate();
        }
        long startTime = SystemClock.elapsedRealtime();

        while (this.location != Location.UNKNOWN && linearOpMode.opModeIsActive() && (SystemClock.elapsedRealtime() - startTime) < 3000) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() == 2) {
                        int goldXPos = -1;
                        int firstSilverXPos = -1;
                        int secondSilverXPos = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldXPos = (int) recognition.getLeft();
                                isGoldInFrame = true;
                            } else if (firstSilverXPos == -1) {
                                firstSilverXPos = (int) recognition.getLeft();
                            } else if(secondSilverXPos == -1){
                                secondSilverXPos = (int) recognition.getLeft();
                            }
                        }
                            if (goldXPos > firstSilverXPos && isGoldInFrame) {
                                this.location = Location.CENTER;
                                tfod.shutdown();
                                return location;
                            } else if (goldXPos < firstSilverXPos && isGoldInFrame) {
                                this.location = Location.LEFT;
                                tfod.shutdown();
                                return location;
                            } else {
                                this.location = Location.RIGHT;
                                tfod.shutdown();
                                return location;
                            }
                    }
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
        location = Location.UNKNOWN;
        return location;
    }

    public void initVuforia() throws VuforiaException {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;
        parameters.cameraDirection = CameraDirection.BACK;

        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);




    }
}
