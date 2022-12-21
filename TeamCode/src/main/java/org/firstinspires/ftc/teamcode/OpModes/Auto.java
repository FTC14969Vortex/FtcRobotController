
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helper.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Test", group = "Concept")
public class Auto extends LinearOpMode {

    private static final String tfodModel = "playPower.tflite";
    private static final String tfodPath = "/sdcard/FIRST/tflitemodels/" + tfodModel;

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    final String[] vortexLabels = {
            "VO",
            "RT",
            "EX"
    };


    private static final String VUFORIA_KEY =
            "AWtcstb/////AAABmfYaB2Q4dURcmKS8qV2asrhnGIuQxM/ioq6TnYqZseP/c52ZaYTjs4/2xhW/91XEaX7c3aw74P3kGZybIaXued3nGShb7oNQyRkVePnFYbabnU/G8em37JQrH309U1zOYtM3bEhRej91Sq6cf6yLjiSXJ+DxxLtSgWvO5f+wM3Wny8MbGUpVSiogYnI7UxEz8OY88d+hgal9u3GhhISdnNucsL+fRAE8mKwT1jGDgUVE1uAJoZFvo95AJWS2Yhdq/N/HpxEH3sBXEm99ci+mdQsl0m96PMCDfV5RgWBjhLbBEIJyQ/xKAbw5Yfr/AKCeB86WDPhR3+Mr8BUvsrycZA6FDJnN5sZZwTg0ZE22+gFL";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;
    public int parkingTarget = 2;
    Robot robot = new Robot();

    public enum AutoSteps {
        detectSignal, goToPole, deliver, park, endAuto
    }

    public AutoSteps Step = AutoSteps.detectSignal;

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        robot.init(hardwareMap);

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.25, 16.0 / 9.0);
        }


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("FL Motor Encoder", robot.FLMotor.getCurrentPosition());
        telemetry.addData("BL Motor Encoder", robot.BLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Encoder", robot.BRMotor.getCurrentPosition());
        telemetry.addData("FR Motor Encoder", robot.FRMotor.getCurrentPosition());
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
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
                            String[] detectLabels = new String[2];
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
                            }
                            if (objectLabel == detectLabels[0]) {
                                parkingTarget = 1;
                            } else if (objectLabel == detectLabels[1]) {
                                parkingTarget = 2;
                            } else if (objectLabel == detectLabels[2]) {
                                parkingTarget = 3;
                            }
                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                            telemetry.addData("Robot Location", robot.Location);
                        }
                        telemetry.update();
                    }
                }

                switch (Step) {
                    case detectSignal:
                        telemetry.addData("Parking Target ", parkingTarget);
                        telemetry.update();
                        Step = AutoSteps.goToPole;
                        break;

                    //Not using this
                    case goToPole:
                        robot.DriveToPosition(0.3, 40, 70);
                        Step = AutoSteps.deliver;

                        //Not using this
                    case deliver:
                        //Put Slider code here

                        //place holder
                        robot.movePulleyToTarget(3, 0.5);
                        sleep(3000);
                        Step = AutoSteps.park;

                    case park:
                        Park(parkingTarget);
                        Step = AutoSteps.endAuto;
                        break;

                    case endAuto:
                        telemetry.addData("➡️", "Auto Finished");
                        telemetry.update();
                        break;
                }
            }
        }
    }


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
            default:
                //    throw new Error("That isn't a valid TFLite model. Maybe check if it's uploaded, or if you made a typo?");
        }
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    private void Park(int location) {
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
