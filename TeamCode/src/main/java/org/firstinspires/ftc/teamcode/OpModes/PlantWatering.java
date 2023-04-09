package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helper.Robot;
import org.firstinspires.ftc.teamcode.OpModes.ReadCSV;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "PlantWatering", group = "Auto")

public class PlantWatering extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;
    Robot robot = new Robot();

    public enum AutoSteps {
        moveToPlant, deliverWater, goBackToStart, endAuto
    }

    public AutoSteps Steps = AutoSteps.moveToPlant;
    double wheelToTubeDist = 8;

    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        robot.init(hardwareMap);
        robot.initVuforia();
        robot.initTfod();

        telemetry.addData("Plant coordinates", Arrays.toString(ReadCSV.PlantCoordinate));

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (robot.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            float leftCoordinate = recognition.getLeft();
                            telemetry.addData("Left Coordinate", leftCoordinate);
                        }
                        telemetry.update();
                    }
                }

                switch (Steps) {
                    case moveToPlant:
                        robot.chassis.DriveToPosition(0.25,ReadCSV.x_distance, (int) (ReadCSV.y_distance - wheelToTubeDist),true);
                        robot.chassis.DriveToPosition(0.25,0, 5,true);
                        Steps = AutoSteps.deliverWater;
                        break;

                    case deliverWater:
                        robot.claw.servo.setPosition(0);
                        sleep(3000);
                        robot.claw.servo.setPosition(1);
                        sleep(2000);
                        Steps = AutoSteps.goBackToStart;
                        break;
                    case goBackToStart:
                        robot.chassis.DriveToPosition(0.25,-ReadCSV.x_distance,-ReadCSV.y_distance,true);
                        Steps = AutoSteps.endAuto;
                        break;
                    case endAuto:
                        telemetry.addData("➡️", "Auto Finished");
                        telemetry.update();
                        break;
                }
            }
        }
    }
}
