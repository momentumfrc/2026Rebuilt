package frc.robot.shootutils;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Scanner;

import com.google.gson.Gson;

/**
 * A holder for the hood angles for each distance.
 */
public class HoodSerializedInformationHolder {

    private final HashMap<Double, Double> angles;

    public HoodSerializedInformationHolder(Map<Double, Double> entries) {
        angles = new HashMap<>(entries);
    }

    public static HoodSerializedInformationHolder fromFile() {
        Gson gson = new Gson();
        try (InputStream stream = HoodSerializedInformationHolder.class.getResourceAsStream("/shootutils/hood_angles.json")) {
            return gson.fromJson(new InputStreamReader(stream), HoodSerializedInformationHolder.class);
        } catch (Exception e) {
            throw new RuntimeException("Could not find the shootutils/hood_angles.json file: " + e.getStackTrace());
        }
    }

    public double getAngle(double distance) {
        List<Double> distances = new ArrayList<>(angles.keySet());
        distances.sort((d1, d2) -> {return (int) Math.round(d1 - d2);});
        double lowerBound = distances.get(0);
        double upperBound = distances.get(distances.size() - 1);
        int i = 0;
        for (double d : distances) {
            if (distance > d) {
                lowerBound = d;
                upperBound = distances.get(i + 1);
            }
            i++;
        }
        double m = (angles.get(upperBound) - angles.get(lowerBound)) / (upperBound - lowerBound);
        return angles.get(lowerBound) + (m * (distance - lowerBound));
    }

}
