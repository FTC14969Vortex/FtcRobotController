package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Plant Info", group = "Auto")

public class PlantInfo extends LinearOpMode{
    public static int plant1_x_distance = 60;
    public static int plant1_y_distance = 30;

    public static int plant2_x_distance = -163;
    public static int plant2_y_distance = -5;

//    public static int plantHeight;
//    public static String plantType;
////    public static int[] PlantCoordinate = {plant1_x_distance, plant1_y_distance};
//    public static String line;

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
    }
}
