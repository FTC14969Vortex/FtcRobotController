package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.Robot;

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

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                switch (Steps) {
                    case moveToPlant:
                        robot.chassis.DriveToPosition(0.4, PlantInfo.plant1_x_distance, (int) (PlantInfo.plant1_y_distance - wheelToTubeDist),true);
                        robot.chassis.DriveToPosition(0.4,0, 5,true);
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
                        robot.chassis.DriveToPosition(0.4,-PlantInfo.plant1_x_distance, (int) (-PlantInfo.plant1_y_distance + wheelToTubeDist),true);
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
