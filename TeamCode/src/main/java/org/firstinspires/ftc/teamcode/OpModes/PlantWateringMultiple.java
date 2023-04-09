package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.Robot;

@Autonomous(name = "Plant Watering Multi", group = "Auto")

public class PlantWateringMultiple extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;
    Robot robot = new Robot();

    public enum AutoSteps {
        moveToPlant1, moveToPlant2, goBackToStart, endAuto
    }

    public AutoSteps Steps = AutoSteps.moveToPlant1;
    double wheelToTubeDist = 8;
    public int currentPlant;

    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        robot.init(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                switch (Steps) {
                    case moveToPlant1:
                        robot.chassis.DriveToPosition(0.4, PlantInfo.plant1_x_distance, (int) (PlantInfo.plant1_y_distance - wheelToTubeDist),true);
                        robot.chassis.DriveToPosition(0.4,0, 5,true);
                        currentPlant = 1;
                        Water();
                        Steps = AutoSteps.moveToPlant2;
                        break;

                    case moveToPlant2:

                        robot.chassis.DriveToPosition(0.4, PlantInfo.plant2_x_distance, (int) (PlantInfo.plant2_y_distance - wheelToTubeDist),true);
                        robot.chassis.DriveToPosition(0.4,0, 25,true);
                        Water();
                        Steps = AutoSteps.goBackToStart;
                        break;

                    case goBackToStart:
                        telemetry.addLine("Going back to start");
                        telemetry.update();

                        robot.chassis.DriveToPosition(0.4, 90, -25, true);
                        Steps = AutoSteps.endAuto;
                        break;
                    case endAuto:
                        telemetry.addData("➡️", "Auto Finished");
                        telemetry.update();
                        robot.chassis.stopDriveMotors();
                        break;
                }
            }
        }
    }
    public void Water() {
        robot.claw.servo.setPosition(0);
        sleep(3000);
        robot.claw.servo.setPosition(1);
        sleep(2000);

    }
}
