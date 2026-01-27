package frc.robot.shootutils;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

/**
 * A holder for the hood angles for each distance.
 */
public class HoodSerializedInformationHolder {

    private final ArrayList<Entry> entries;

    private static HoodSerializedInformationHolder instance = null;

    @JsonCreator
    public HoodSerializedInformationHolder(@JsonProperty("entries") List<Entry> entries) {
        this.entries = new ArrayList<>(entries);
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
        List<Double> distances = new ArrayList<>(entries.stream()
                .map((entry) -> {
                    return entry.getDistance();
                })
                .toList());
        List<Double> angles = new ArrayList<>(entries.stream()
                .map((entry) -> {
                    return entry.getHoodAngle();
                })
                .toList());
        distances.sort((d1, d2) -> {
            return (int) Math.round(d1 - d2);
        });
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
        List<Double> distances = new ArrayList<>(entries.stream()
                .map((entry) -> {
                    return entry.getDistance();
                })
                .toList());
        List<Double> speeds = new ArrayList<>(entries.stream()
                .map((entry) -> {
                    return entry.getFlywheelSpeed();
                })
                .toList());
        distances.sort((d1, d2) -> {
            return (int) Math.round(d1 - d2);
        });
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

    public static class Entry {

        private double distance;
        private double flywheelSpeed;
        private double hoodAngle;

        @JsonCreator
        public Entry(
                @JsonProperty("distance") double distance,
                @JsonProperty("flywheelSpeed") double flywheelSpeed,
                @JsonProperty("hoodAngle") double hoodAngle) {
            this.distance = distance;
            this.flywheelSpeed = flywheelSpeed;
            this.hoodAngle = hoodAngle;
        }

        public double getFlywheelSpeed() {
            return flywheelSpeed;
        }

        public double getDistance() {
            return distance;
        }

        public double getHoodAngle() {
            return hoodAngle;
        }
    }
}
