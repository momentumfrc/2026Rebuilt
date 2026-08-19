package frc.robot.shootutils;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.MutAngle;
import org.wpilib.units.measure.MutAngularVelocity;
import org.wpilib.units.measure.MutTime;
import org.wpilib.units.measure.Time;
import org.wpilib.system.Filesystem;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Comparator;
import java.util.List;

/**
 * A holder for the hood angles for each distance.
 */
public class HoodSerializedInformationHolder {
    private static final File DATA_FILE = new File(Filesystem.getDeployDirectory(), "shooter-data.json");

    public static final DistanceUnit DISTANCE_STORE_UNIT = Units.Meters;
    public static final AngleUnit HOOD_ANGLE_STORE_UNIT = Units.Degrees;
    public static final AngularVelocityUnit FLYWHEEL_SPEED_STORE_UNIT = Units.RPM;
    public static final TimeUnit TIME_OF_FLIGHT_STORE_UNIT = Units.Seconds;

    private final List<Entry> entries;

    private static HoodSerializedInformationHolder instance = null;

    private final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    private MutAngle hoodAngle = HOOD_ANGLE_STORE_UNIT.mutable(0);
    private MutAngularVelocity flywheelSpeed = FLYWHEEL_SPEED_STORE_UNIT.mutable(0);
    private MutTime timeOfFlight = TIME_OF_FLIGHT_STORE_UNIT.mutable(0);

    // package-private for testing
    @JsonCreator
    HoodSerializedInformationHolder(@JsonProperty("entries") List<Entry> entries) {
        this.entries = entries.stream()
                .sorted(Comparator.comparingDouble((entry) -> entry.distance()))
                .toList();

        for (var entry : this.entries) {
            if (entry.hoodAngle() != null) {
                hoodAngleMap.put(entry.distance(), entry.hoodAngle());
            }
            if (entry.flywheelSpeed() != null) {
                flywheelSpeedMap.put(entry.distance(), entry.flywheelSpeed());
            }
            if (entry.timeOfFlight() != null) {
                timeOfFlightMap.put(entry.distance(), entry.timeOfFlight());
            }
        }
    }

    public static HoodSerializedInformationHolder getInstance() {
        if (instance == null) {
            instance = fromFile();
        }
        return instance;
    }

    private static HoodSerializedInformationHolder fromFile() {
        ObjectMapper mapper = new ObjectMapper();

        if (DATA_FILE.canRead() == false) {
            throw new IllegalStateException("Could not open data file for reading at [" + DATA_FILE.toString() + "]");
        }

        try (InputStream stream = new FileInputStream(DATA_FILE); ) {
            return mapper.readValue(stream, HoodSerializedInformationHolder.class);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    static record Entry(Double distance, Double hoodAngle, Double flywheelSpeed, Double timeOfFlight) {
        @JsonCreator
        public Entry(
                @JsonProperty("distance") Double distance,
                @JsonProperty("hoodAngle") Double hoodAngle,
                @JsonProperty("flywheelSpeed") Double flywheelSpeed,
                @JsonProperty("timeOfFlight") Double timeOfFlight) {
            this.distance = distance;
            this.flywheelSpeed = flywheelSpeed;
            this.hoodAngle = hoodAngle;
            this.timeOfFlight = timeOfFlight;
        }
    }

    /**
     * Get the interpolated hood angle.
     */
    public Angle getHoodAngle(Distance distanceToTarget) {
        return hoodAngle.mut_replace(hoodAngleMap.get(distanceToTarget.in(DISTANCE_STORE_UNIT)), HOOD_ANGLE_STORE_UNIT);
    }

    /**
     * Get the interpolated hood angle, in {@link #HOOD_ANGLE_STORE_UNIT} units.
     * @param distanceToTarget the distance to the target in {@link #DISTANCE_STORE_UNIT} units.
     */
    public double getHoodAngle(double distanceToTarget) {
        return hoodAngleMap.get(distanceToTarget);
    }

    /**
     * Get the interpolated flywheel speed.
     */
    public AngularVelocity getFlywheelSpeed(Distance distanceToTarget) {
        return flywheelSpeed.mut_replace(
                flywheelSpeedMap.get(distanceToTarget.in(DISTANCE_STORE_UNIT)), FLYWHEEL_SPEED_STORE_UNIT);
    }

    /**
     * Get the interpolated hood angle, in {@link #FLYWHEEL_SPEED_STORE_UNIT} units.
     * @param distanceToTarget the distance to the target in {@link #DISTANCE_STORE_UNIT} units.
     */
    public double getFlywheelSpeed(double distanceToTarget) {
        return flywheelSpeedMap.get(distanceToTarget);
    }

    /**
     * Get the interpolated time of flight.
     */
    public Time getTimeOfFlight(Distance distanceToTarget) {
        return timeOfFlight.mut_replace(
                timeOfFlightMap.get(distanceToTarget.in(DISTANCE_STORE_UNIT)), TIME_OF_FLIGHT_STORE_UNIT);
    }

    /**
     * Get the interpolated time of flight, in {@link #TIME_OF_FLIGHT_STORE_UNIT} units.
     * @param distanceToTarget the distance to the target in {@link #DISTANCE_STORE_UNIT} units.
     */
    public double getTimeOfFlight(double distanceToTarget) {
        return timeOfFlightMap.get(distanceToTarget);
    }
}
