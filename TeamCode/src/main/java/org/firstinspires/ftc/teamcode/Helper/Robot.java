package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Related to IMU

// Related to vision
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class Robot {
    /*
    Properties that describe hardware.
     */
    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;

    public WaterDispenser claw = new WaterDispenser();
    public Arm arm = new Arm();
    public Chassis chassis = new Chassis();
    public VSlider vSlider = new VSlider();

    private int robotInUse = 2022;

    /** This is code to set up TenserFlow and Vuforia

     * We have a model trained, but are currently not using it, as we could not integrate it in our 24 hours.
     * We have left the code here so that in the future we can implement navigation using our custom model

     **/

//    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
//    public static final String[] LABELS = {
//            "Arrows",
//            "Balloons",
//            "Bars"
//    };
//
//    private static final String VUFORIA_KEY =
//            "AWtcstb/////AAABmfYaB2Q4dURcmKS8qV2asrhnGIuQxM/ioq6TnYqZseP/c52ZaYTjs4/2xhW/91XEaX7c3aw74P3kGZybIaXued3nGShb7oNQyRkVePnFYbabnU/G8em37JQrH309U1zOYtM3bEhRej91Sq6cf6yLjiSXJ+DxxLtSgWvO5f+wM3Wny8MbGUpVSiogYnI7UxEz8OY88d+hgal9u3GhhISdnNucsL+fRAE8mKwT1jGDgUVE1uAJoZFvo95AJWS2Yhdq/N/HpxEH3sBXEm99ci+mdQsl0m96PMCDfV5RgWBjhLbBEIJyQ/xKAbw5Yfr/AKCeB86WDPhR3+Mr8BUvsrycZA6FDJnN5sZZwTg0ZE22+gFL";
//    public VuforiaLocalizer vuforia; //vuforia object stored in vision class.
//    public TFObjectDetector tfod; //tfod object stored in vision class.


    HardwareMap hwMap = null;


    public void init(HardwareMap ahwMap) throws InterruptedException {
        hwMap = ahwMap;
        claw.init(hwMap);
        chassis.init(hwMap);
        arm.init(hwMap);
        vSlider.init(hwMap);

        if (robotInUse == 2021) {
            arm.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            vSlider.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            claw.servo.setDirection(Servo.Direction.FORWARD);
        } else if (robotInUse == 2022) {
            arm.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            vSlider.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            claw.servo.setDirection(Servo.Direction.REVERSE);
        }


    }

//    public void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = CameraDirection.BACK;
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//    }
//
//    public void initTfod() {
//        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.7f;
//        tfodParameters.isModelTensorFlow2 = true;
//        tfodParameters.inputSize = 300;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//
//        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
//        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
//        // chassis.tfod.loadModelFromFile(tfodPath, LABELS);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
//
//    }
}

