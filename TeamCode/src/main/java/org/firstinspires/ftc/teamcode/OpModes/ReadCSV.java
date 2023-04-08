package org.firstinspires.ftc.teamcode.OpModes;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;

public class ReadCSV {
    public static int x_distance = 0;
    public static int y_distance = 0;
    public static int plantHeight;
    public static String plantType;
    public static int[] PlantCoordinate = { x_distance, y_distance };

    public static void UseValues(String[] val) {
        x_distance = Integer.parseInt(val[0]);
        y_distance = Integer.parseInt(val[1]);
        plantHeight = Integer.parseInt(val[2]);
        plantType = val[3];

        PlantCoordinate[0] = x_distance;
        PlantCoordinate[1] = y_distance;
    }

    public void ReadData() {
        String file = "/Users/shash/Desktop/Plants.csv";
        String line;
        BufferedReader reader = null;
        try {

            reader = new BufferedReader(new FileReader(file));

            while ((line = reader.readLine()) != null) {
                System.out.println(line);
                String[] values = line.split(",");
                UseValues(values);
            }
            reader.close();

        } catch (FileNotFoundException ex) {
            ex.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
