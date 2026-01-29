package frc.robot.shootutils;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.InputStream;
import java.util.Comparator;
import java.util.List;

/**
 * A holder for the hood angles for each distance.
 */
public class HoodSerializedInformationHolder {

    private final List<Entry> entries;

    private static HoodSerializedInformationHolder instance = null;

    @JsonCreator
    private HoodSerializedInformationHolder(@JsonProperty("entries") List<Entry> entries) {
        this.entries = entries.stream()
                .sorted(Comparator.comparingDouble((entry) -> entry.distance()))
                .toList();
    }

    public static HoodSerializedInformationHolder create() {
        if (instance == null) {
            instance = fromFile();
        }
        return instance;
    }

    private static HoodSerializedInformationHolder fromFile() {
        ObjectMapper mapper = new ObjectMapper();
        InputStream stream =
                Thread.currentThread().getContextClassLoader().getResourceAsStream("shootutils/hood_angles.json");

        if (stream == null) {
            throw new IllegalStateException("Could not find shootutils/hood_angles.json");
        }

        try {
            return mapper.readValue(stream, HoodSerializedInformationHolder.class);
        } catch (Exception e) {
            throw new RuntimeException("Could not parse the shootutils/hood_angles.json file", e);
        }
    }

    public double getAngle(double distance) {
        List<Double> distances = entries.stream().map(entry -> entry.distance()).toList();
        List<Double> angles = entries.stream().map(entry -> entry.hoodAngle()).toList();
        int lowerBound = 0;
        int upperBound = distances.size() - 1;
        if (distance < distances.get(lowerBound)) {
            upperBound = 1;
        } else if (distance > distances.get(upperBound)) {
            lowerBound = distances.size() - 2;
        } else {
            int i = 0;
            for (double d : distances) {
                if (distance > d) {
                    lowerBound = i;
                    upperBound = i + 1;
                }
                i++;
            }
        }
        double m = (angles.get(upperBound) - angles.get(lowerBound))
                / (distances.get(upperBound) - distances.get(lowerBound));
        return angles.get(lowerBound) + (m * (distance - distances.get(lowerBound)));
    }

    public double getFlywheelSpeed(double distance) {
        List<Double> distances = entries.stream().map(entry -> entry.distance()).toList();
        List<Double> speeds =
                entries.stream().map(entry -> entry.flywheelSpeed()).toList();
        int lowerBound = 0;
        int upperBound = distances.size() - 1;
        if (distance < distances.get(lowerBound)) {
            upperBound = 1;
        } else if (distance > distances.get(upperBound)) {
            lowerBound = distances.size() - 2;
        } else {
            int i = 0;
            for (double d : distances) {
                if (distance > d) {
                    lowerBound = i;
                    upperBound = i + 1;
                }
                i++;
            }
        }
        double m = (speeds.get(upperBound) - speeds.get(lowerBound))
                / (distances.get(upperBound) - distances.get(lowerBound));
        return speeds.get(lowerBound) + (m * (distance - distances.get(lowerBound)));
    }

    public static record Entry(double distance, double flywheelSpeed, double hoodAngle) {

        @JsonCreator
        public Entry(
                @JsonProperty("distance") double distance,
                @JsonProperty("flywheelSpeed") double flywheelSpeed,
                @JsonProperty("hoodAngle") double hoodAngle) {
            this.distance = distance;
            this.flywheelSpeed = flywheelSpeed;
            this.hoodAngle = hoodAngle;
        }
    }
}
