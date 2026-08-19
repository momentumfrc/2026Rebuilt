package first.shootutils;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

import io.avaje.jsonb.Json;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;

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

    // package-private for testing
    @Json.Creator
    HoodSerializedInformationHolder(List<Entry> entries) {
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
        Jsonb mapper = Jsonb.builder().build();

        JsonType<HoodSerializedInformationHolder> type = mapper.type(HoodSerializedInformationHolder.class);

        if (DATA_FILE.canRead() == false) {
            throw new IllegalStateException("Could not open data file for reading at [" + DATA_FILE.toString() + "]");
        }

        try (InputStream stream = new FileInputStream(DATA_FILE); ) {
            return type.fromJson(stream);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Json
    static record Entry(Double distance, Double hoodAngle, Double flywheelSpeed, Double timeOfFlight) {
        public Entry(
                 Double distance,
                 Double hoodAngle,
                 Double flywheelSpeed,
                 Double timeOfFlight) {
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
        return HOOD_ANGLE_STORE_UNIT.of(hoodAngleMap.get(distanceToTarget.in(DISTANCE_STORE_UNIT)));
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
        return FLYWHEEL_SPEED_STORE_UNIT.of(flywheelSpeedMap.get(distanceToTarget.abs(DISTANCE_STORE_UNIT)));
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
        return TIME_OF_FLIGHT_STORE_UNIT.of(timeOfFlightMap.get(distanceToTarget.abs(DISTANCE_STORE_UNIT)));
    }

    /**
     * Get the interpolated time of flight, in {@link #TIME_OF_FLIGHT_STORE_UNIT} units.
     * @param distanceToTarget the distance to the target in {@link #DISTANCE_STORE_UNIT} units.
     */
    public double getTimeOfFlight(double distanceToTarget) {
        return timeOfFlightMap.get(distanceToTarget);
    }
}
