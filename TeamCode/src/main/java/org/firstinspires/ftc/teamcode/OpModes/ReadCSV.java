package org.firstinspires.ftc.teamcode.OpModes;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.InputStream;
import java.util.Arrays;

@Autonomous(name = "ReadCSV", group = "Auto")

public class ReadCSV extends LinearOpMode{
    public static int x_distance = 60;
    public static int y_distance = 30;
    public static int plantHeight;
    public static String plantType;
    public static int[] PlantCoordinate = { x_distance, y_distance };
    public static String[] values;
    public static String line;

//    public static void UseValues(String[] val) {
//        x_distance = Integer.parseInt(val[0]);
//        y_distance = Integer.parseInt(val[1]);
//
//        PlantCoordinate[0] = x_distance;
//        PlantCoordinate[1] = y_distance;
//    }
//
//    public static void ReadData() {
//        String file = "Test.txt";
//        BufferedReader reader = null;
//        try {
//
//            reader = new BufferedReader(new FileReader(file));
//
////            while ((line = reader.readLine()) != null) {
////                System.out.println(line);
////                values = line.split(",");
////            }
//            line = reader.readLine();
//            reader.close();
//
//        } catch (FileNotFoundException ex) {
//            ex.printStackTrace();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData(">", line);
                telemetry.update();
            }
        }
    }
}
