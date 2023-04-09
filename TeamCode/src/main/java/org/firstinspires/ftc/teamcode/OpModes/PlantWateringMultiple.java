package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helper.Robot;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Plant Watering Multi", group = "Auto")

public class PlantWateringMultiple extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;
    Robot robot = new Robot();

    public enum AutoSteps {
        moveToPlant1, deliverWaterTo1, moveToPlant2, deliverWater, goBackToStart, endAuto
    }

    public AutoSteps Steps = AutoSteps.moveToPlant1;
    double wheelToTubeDist = 8;
    public int currentPlant;

    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        robot.init(hardwareMap);
        robot.initVuforia();
        robot.initTfod();

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
                    case moveToPlant1:
                        robot.chassis.DriveToPosition(0.25,ReadCSV.plant1_x_distance, (int) (ReadCSV.plant1_y_distance - wheelToTubeDist),true);
                        robot.chassis.DriveToPosition(0.25,0, 5,true);
                        currentPlant = 1;
                        Steps = AutoSteps.deliverWater;
                        break;

                    case deliverWater:
                        robot.claw.servo.setPosition(0);
                        sleep(3000);
                        robot.claw.servo.setPosition(1);
                        sleep(2000);

                        if(currentPlant == 1) {
                          Steps = AutoSteps.moveToPlant2;
                        } else if(currentPlant == 2) {
                            Steps = AutoSteps.goBackToStart;
                        }
                        break;

                    case moveToPlant2:

                        robot.chassis.DriveToPosition(0.25,ReadCSV.plant2_x_distance, (int) (ReadCSV.plant2_y_distance - wheelToTubeDist),true);
                        robot.chassis.DriveToPosition(0.25,0, 5,true);
                        Steps = AutoSteps.deliverWater;
                        break;

                    case goBackToStart:
                        robot.chassis.DriveToPosition(0.25,-ReadCSV.plant2_x_distance,-ReadCSV.plant2_y_distance,true);
                        robot.chassis.DriveToPosition(0.25,-ReadCSV.plant1_x_distance,-ReadCSV.plant1_y_distance,true);
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
