package frc.robot.shootutils;

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
    private final HashMap<Double, Double> flywheelSpeeds;

    private static HoodSerializedInformationHolder instance = null;

    private HoodSerializedInformationHolder(Map<Double, Double> angleEntries, Map<Double, Double> flywheelEntries) {
        angles = new HashMap<>(angleEntries);
        flywheelSpeeds = new HashMap<>(flywheelEntries);
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
        if (distance < lowerBound) {
            upperBound = distances.get(1);
        } else if (distance > upperBound) {
            lowerBound = distances.get(distances.size() - 2);
        } else {
            int i = 0;
            for (double d : distances) {
                if (distance > d) {
                    lowerBound = d;
                    upperBound = distances.get(i + 1);
                }
                i++;
            }
        }
        double m = (angles.get(upperBound) - angles.get(lowerBound)) / (upperBound - lowerBound);
        return angles.get(lowerBound) + (m * (distance - lowerBound));
    }

    public double getFlywheelSpeed(double distance) {
        List<Double> distances = new ArrayList<>(flywheelSpeeds.keySet());
        distances.sort((d1, d2) -> {
            return (int) Math.round(d1 - d2);
        });
        double lowerBound = distances.get(0);
        double upperBound = distances.get(distances.size() - 1);
        if (distance < lowerBound) {
            upperBound = distances.get(1);
        } else if (distance > upperBound) {
            lowerBound = distances.get(distances.size() - 2);
        } else {
            int i = 0;
            for (double d : distances) {
                if (distance > d) {
                    lowerBound = d;
                    upperBound = distances.get(i + 1);
                }
                i++;
            }
        }
        double m = (flywheelSpeeds.get(upperBound) - flywheelSpeeds.get(lowerBound)) / (upperBound - lowerBound);
        return flywheelSpeeds.get(lowerBound) + (m * (distance - lowerBound));
    }
}
