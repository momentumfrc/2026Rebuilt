package frc.robot.shootutils;

import com.fasterxml.jackson.annotation.JsonFormat.Feature;
import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A holder for the hood angles for each distance.
 */
public class HoodSerializedInformationHolder {

    private final HashMap<Double, Double> angles;

    private static HoodSerializedInformationHolder instance = null;

    private HoodSerializedInformationHolder(Map<Double, Double> entries) {
        angles = new HashMap<>(entries);
    }

    public static HoodSerializedInformationHolder create() {
        if (instance == null) {
            instance = fromFile();
        }
        return instance;
    }

    private static HoodSerializedInformationHolder fromFile() {
        ObjectMapper mapper = new ObjectMapper();
        try (InputStream stream =
                HoodSerializedInformationHolder.class.getResourceAsStream("/shootutils/hood_angles.json")) {
            return mapper.readValue(new InputStreamReader(stream), HoodSerializedInformationHolder.class);
        } catch (Exception e) {
            throw new RuntimeException("Could not find the shootutils/hood_angles.json file: " + e.getStackTrace());
        }
    }

    public double getAngle(double distance) {
        List<Double> distances = new ArrayList<>(angles.keySet());
        distances.sort((d1, d2) -> {
            return (int) Math.round(d1 - d2);
        });
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
