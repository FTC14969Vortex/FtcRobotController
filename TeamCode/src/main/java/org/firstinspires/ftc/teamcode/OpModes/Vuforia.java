
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helper.Robot;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Vuforia_test", group = "Vuforia")

public class Vuforia extends LinearOpMode {

    //Variables:
    public int parkingTarget = 3;
    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;

    Robot robot = new Robot();

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String tfodModel = "jan2022polesAndSignals.tflite";
    private static final String tfodPath = "/sdcard/FIRST/tflitemodels/" + tfodModel;


    final String[] vortexLabels = {
            "VO",
            "RT",
            "EX"
    };
    final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    final String[] poleLABELS = {
            "arrow",
            "balloons",
            "bars",
            "pole"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AWtcstb/////AAABmfYaB2Q4dURcmKS8qV2asrhnGIuQxM/ioq6TnYqZseP/c52ZaYTjs4/2xhW/91XEaX7c3aw74P3kGZybIaXued3nGShb7oNQyRkVePnFYbabnU/G8em37JQrH309U1zOYtM3bEhRej91Sq6cf6yLjiSXJ+DxxLtSgWvO5f+wM3Wny8MbGUpVSiogYnI7UxEz8OY88d+hgal9u3GhhISdnNucsL+fRAE8mKwT1jGDgUVE1uAJoZFvo95AJWS2Yhdq/N/HpxEH3sBXEm99ci+mdQsl0m96PMCDfV5RgWBjhLbBEIJyQ/xKAbw5Yfr/AKCeB86WDPhR3+Mr8BUvsrycZA6FDJnN5sZZwTg0ZE22+gFL";

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


    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        //Also initialize the robot
        robot.init(hardwareMap);


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        File f = new File(tfodPath);
        if(f.exists()) { telemetry.addData(">", tfodPath + " exists"); }
        else {telemetry.addData(">", tfodPath + " does not exist"); }
        telemetry.addData(">", "Good luck!");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive() && !isStopRequested()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());
                            String objectLabel = recognition.getLabel();
                            String[] detectLabels = new String[3];
                            int i = 0;
                            switch(tfodModel) {
                                case "modelvortex.tflite":
                                    for (String label : vortexLabels) {
                                        detectLabels[i] = label;
                                        i++;
                                    }
                                break;

                                case "playPower.tflite":
                                    for (String label : LABELS) {
                                        detectLabels[i] = label;
                                        i++;
                                    }
                                    break;
                                case "dec2022Junction.tflite":
                                case "jan2022polesandsignals.tflite":
                                    for (String label : poleLABELS) {
                                    detectLabels[i] = label;
                                    i++;
                                }
                                break;
                            }
                            for(int j = 0; j < 3; j++) {
                                if (objectLabel == detectLabels[j]) {
                                    parkingTarget = j + 1;
                                }
                            }

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                            telemetry.addData("Parking Target", parkingTarget);
                        }

                        telemetry.update();
                    }
                }
                //Skip the parking step
                parkingTarget = 0;
                switch(parkingTarget) {
                    case 1:
                        runtime.reset();
                        timeout_ms = 3000;
                        robot.Drive(0.3, 90);
                        while (opModeIsActive() && (runtime.milliseconds() < timeout_ms) && (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {}
                        robot.stopDriveMotors();
                        sleep(100);

                        runtime.reset();
                        timeout_ms = 3000;
                        robot.Strafe(0.3, -60);
                        while (opModeIsActive() && (runtime.milliseconds() < timeout_ms) && (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {}
                        robot.stopDriveMotors();
                        break;
                    case 2:
                        runtime.reset();
                        timeout_ms = 3000;
                        robot.Drive(0.3, 90);
                        while (opModeIsActive() && (runtime.milliseconds() < timeout_ms) && (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {}
                        robot.stopDriveMotors();
                        break;
                    case 3:
                        runtime.reset();
                        timeout_ms = 3000;
                        robot.Drive(0.3, 90);
                        while (opModeIsActive() && (runtime.milliseconds() < timeout_ms) && (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {}
                        robot.stopDriveMotors();
                        sleep(100);

                        runtime.reset();
                        timeout_ms = 3000;
                        robot.Strafe(0.3, 60);
                        while (opModeIsActive() && (runtime.milliseconds() < timeout_ms) && (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {}
                        robot.stopDriveMotors();
                        break;
                }
            }
        }
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


    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.

        switch(tfodModel) {
            case "modelvortex.tflite":
                tfod.loadModelFromFile(tfodPath, vortexLabels);
                break;
            case "playPower.tflite":
                tfod.loadModelFromFile(tfodPath, LABELS);
                break;
            case "dec2022Junction.tflite":
            case "jan2022polesAndSignals.tflite":
                tfod.loadModelFromFile(tfodPath,poleLABELS);
                break;
            default:
            //    throw new Error("That isn't a valid TFLite model. Maybe check if it's uploaded, or if you made a typo?");
        }
    }
}


