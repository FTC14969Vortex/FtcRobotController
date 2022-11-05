package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helper.Robot;


@Autonomous
public class PowerPlay extends LinearOpMode {

   public String stateMachine = "Detect";
   /*
   1: Collect[Cone]
   2: Move[to Pole]
   3: Place[Cone]
   (repeat 1-3 as many times as possible)
   4: Go[to Signal]
   5: Park
    */
   public int conesPlaced = 0;
   public int quota = 3;

   public int parkingTarget = 3;
   private ElapsedTime runtime = new ElapsedTime();
   double timeout_ms = 0;

   Robot robot = new Robot();

   private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
   // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

   private static final String[] LABELS = {
           "1 Bolt",
           "2 Bulb",
           "3 Panel"
   };

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
       //Init
       if(!initVuforia()) {throw new Error("Vuforia failed to init");}
       if(!initTfod()) {throw new Error("Tfod failed to init");}
       robot.init(hardwareMap);
       if (tfod != null) {
           tfod.activate();
           tfod.setZoom(1.0, 16.0/9.0);
       }

       log(">", "Press play for bugs :D");
       waitForStart();
       log(">", "Here they come");
           while (opModeIsActive()) {
               log(">","State changing");
               switch(stateMachine) {
                   case "Detect":
                       log(">","Starting the 'Detect' state");
                       List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                       if(updatedRecognitions != null) {
                           log("> : Objects Detected", updatedRecognitions.size());
                           for(Recognition rec : updatedRecognitions) {
                               double col = (rec.getRight() + rec.getLeft()) / 2;
                               double row = (rec.getBottom() + rec.getTop()) / 2;
                               double width = Math.abs(rec.getRight() - rec.getLeft());
                               double height = Math.abs(rec.getTop() - rec.getBottom());
                               String objLabel = rec.getLabel();
                               int i = 0;
                               for(String check : LABELS) {
                                   if(objLabel == check) {
                                       parkingTarget = i;
                                   }
                                   i++;
                               }
                               log("", " ");
                               log("Image", "%s (%.0f %% Conf.)", rec.getLabel(), rec.getConfidence() * 100);
                               log("- Position (Row/Col)", "%.0f / %.0f", row, col);
                               log("- Size (Width/Height)", "%.0f / %.0f", width, height);
                               log("Parking Target", parkingTarget);
                           }
                       }
                       stateMachine = "Collect";
                       //break;
                   case "Collect":
                       log(">","Starting the 'Collect' state");
                       robot.turnRobot(90);
                       robot.startDriveToPosition(4,2);
                       robot.useClaw();
                       robot.moveArmToTarget(1,3);
                       robot.moveArmToTarget(1,-3);
                       stateMachine = "Move";
                       robot.turnRobot(90);
                       break;
                   case "Move":
                       log(">","Starting the 'Move' state");
                       //Really just making up numbers here. Need to do some tests to detirmine actual distances
                       robot.startDriveToPosition(10,2);
                       robot.turnRobot(90);
                       robot.startDriveToPosition(10,2);
                       robot.turnRobot(-90);
                       robot.startDriveToPosition(1,2);
                       stateMachine = "Place";
                       break;
                   case "Place":
                       log(">","Starting the 'Place' state");
                      /*
                      Low
                      Med
                      High
                       */
                       String targetPole = "Low";
                       //Raise the Crane
                       switch(targetPole) {
                           //Values are based off the FTC Sim. Have to reevaluate for our purposes.
                           case "Low":
                               robot.raiseCrane(2,200);
                               break;
                           case "Med":
                               robot.raiseCrane(4,700);
                               break;
                           case "High":
                               robot.raiseCrane(6,700);
                               break;
                           default:
                               throw new Error(targetPole + "is not a valid Pole Height");
                       }
                       //Time to Extend the Arm
                       robot.moveArmToTarget(2, 10);
                       robot.useClaw();
                       robot.moveArmToTarget(2,0);
                       robot.raiseCrane(2,0);
                       conesPlaced++;
                       if(conesPlaced >= quota) {
                           stateMachine = "Park";
                       }
                       else {
                           stateMachine = "Collect";
                       }
                       robot.turnRobot(180);
                       robot.startDriveToPosition(10,3);
                       robot.turnRobot(90);
                       robot.startDriveToPosition(10,3);
                       break;
                   case "Park":
                       log(">","Starting the 'Park' state. It should be returned to its starting point");
                       //Code to return to starting position
                       switch(parkingTarget) {
                           case 0:
                               runtime.reset();
                               timeout_ms = 3000;
                               robot.startDriveToPosition(0.3,0.9);
                               while(opModeIsActive() && (runtime.milliseconds() < timeout_ms) && robot.FLMotor.isBusy() && robot.FRMotor.isBusy()) {};
                               robot.stopDriveMotors();
                               sleep(300);
                               break;
                           case 1:
                               runtime.reset();
                               timeout_ms = 3000;
                               robot.startStrafeToPosition(0.3,-60);
                               while(opModeIsActive() && (runtime.milliseconds() < timeout_ms) && robot.FLMotor.isBusy() && robot.FRMotor.isBusy()) {};
                               robot.stopDriveMotors();
                               sleep(300);
                               break;
                           case 2:
                               runtime.reset();
                               timeout_ms = 3000;
                               robot.startDriveToPosition(0.3,90);
                               while(opModeIsActive() && (runtime.milliseconds() < timeout_ms) && robot.FLMotor.isBusy() && robot.FRMotor.isBusy()) {};
                               robot.stopDriveMotors();
                               sleep(300);
                               break;
                           default:
                               throw new Error("Invalid Parking Target");
                       }
                       break;
               }
           }
       }
       public PowerPlay toValhalla() {
           log(">", "Fires are rising and the bells are ringing");
           sleep(100);
           log(">", "Glory take us into Odin's halls");
           sleep(100);
           log(">", "Golden glimmer and the sound of singing");
           sleep(100);
           log(">", "Oh-ho-oh, the echoes of eternity");
           sleep(100);
           log(">", "Oh-ho-oh, Valhalla Calling Me");
           sleep(100);
           log(">", "Oh-ho-oh, to pluck the strings of destiny");
           sleep(100);
           log(">", "VALHALLA CALLING ME");
           sleep(50);
           robot.startDriveToPosition(69,420);
           return this;
       }

   /**
    * Initialize the Vuforia localization engine.
    */
   private boolean initVuforia() {
       /*
        * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        */
       VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

       parameters.vuforiaLicenseKey = VUFORIA_KEY;
       parameters.cameraDirection = CameraDirection.BACK;

       //  Instantiate the Vuforia engine
       vuforia = ClassFactory.getInstance().createVuforia(parameters);
       if(vuforia != null) {
        return true;
       }
       else {
        return false;
       }
   }

   /**
    * Initialize the TensorFlow Object Detection engine.
    */
   private boolean initTfod() {
       int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
               "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.75f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 300;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

       // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
       // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
       // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
       if(tfod != null) {
        return true;
       }
       else {
        return false;
       }
   }
   void log(String caption, String message) {
     telemetry.addData(caption, message);
     telemetry.update();
   }
}
