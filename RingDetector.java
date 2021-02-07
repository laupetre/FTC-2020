 

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class RingDetector  {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private int numberOfRings = 0;


    private static final String VUFORIA_KEY =
            "AaPwcjj/////AAABmRakVCz9mUdkqEeZV/k3mUwSw4UCyd7zKv4kwtUNlKD4Vaw1/h9xUDg6yxfMkXPjZ1Y39fb3mUQF/LOaEm828OSHINnZCiR4bnTTknfgMysNClx2y/fe+qb3/8prwtjNQgfkHiqfVCKTb77Z0m8LWvV1y1AEeLGpbXbubcTwVUSZGJZu+eHkhF7LSO/Bs0ydbVtNRdMX8IRDpJBGCtcBInFtbPUU57kyaFS/45Uv4UKI0dlkivuOEykLsKJe7mxMKArxvVgAAkt9pC/mV+GYReNigc02DxRKEKtMdA4qEW9SrNfsbfBeDZmGr7ZCkAjNg4a9mbSEtyr3/nq0sU54X/zdgW3xGMmWyYpP72WC5F3p";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public int ringNumber () {

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
              //  telemetry.addData("# Object Detected", updatedRecognitions.size());

                //numberOfRings = updatedRecognitions.size();

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {

                    //
                    if (recognition.getLabel() == "Quad"){
                        numberOfRings = 4;
                    }
                    else if (recognition.getLabel() == "Single"){
                        numberOfRings = 1;

                    }
                    else{
                        numberOfRings = 0;
                    }


                   //telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            //recognition.getLeft(), recognition.getTop());
                    //telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                           // recognition.getRight(), recognition.getBottom());
                }
                //telemetry.update();
            }
        }

        return numberOfRings;
    }

    public void init(HardwareMap hardwareMap) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod(hardwareMap);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(1.5, 1.78);
        }

/*
        if (tfod != null) {
            tfod.shutdown();
        }
*/

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public int ringdeDectorLoop(){
        int loops = 0;
        int rings = 0;

        rings = this.ringNumber();

        while (loops < 1000 || rings != 1 || rings != 4 ){
            rings = this.ringNumber();
            loops++;
        }

        return rings;
    }
}
