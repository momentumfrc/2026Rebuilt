package frc.robot.shootutils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Consumer;

public class ShooterEmpiricalDataStore implements NetworkTable.TableEventListener {
    private static final String TABLE_NAME = "shooter-data";

    private static final DistanceUnit DISTANCE_STORE_UNIT = Units.Meters;
    private static final AngleUnit HOOD_ANGLE_STORE_UNIT = Units.Degrees;
    private static final AngularVelocityUnit FLYWHEEL_SPEED_STORE_UNIT = Units.RPM;

    public static record Datum(Distance distance, Angle hoodAngle, AngularVelocity flywheelSpeed) {}

    private static ShooterEmpiricalDataStore instance = null;

    public static ShooterEmpiricalDataStore getInstance() {
        if (instance == null) {
            instance = new ShooterEmpiricalDataStore();
        }
        return instance;
    }

    private NetworkTable table;
    private Map<Distance, Datum> data;

    private Map<Distance, Datum> estimateData;

    public ShooterEmpiricalDataStore() {
        table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
        data = new TreeMap<>();
        estimateData = new TreeMap<>();
        table.addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll, NetworkTableEvent.Kind.kImmediate), this);

        populateInitialData();
    }

    private void populateInitialData() {
        // Best guess from physics calculations
        Consumer<Datum> addInitialData = (datum) -> estimateData.put(datum.distance(), datum);
        addInitialData.accept(new Datum(Units.Inches.of(14), Units.Degrees.of(8.5), Units.RPM.of(1310)));
        addInitialData.accept(new Datum(Units.Inches.of(20), Units.Degrees.of(11), Units.RPM.of(1077.85)));
        addInitialData.accept(new Datum(Units.Inches.of(120), Units.Degrees.of(28), Units.RPM.of(1316.61)));
        addInitialData.accept(new Datum(Units.Inches.of(240), Units.Degrees.of(35), Units.RPM.of(1662.368)));
    }

    public void addDatum(Datum datum) {
        String distanceStr = Double.toString(datum.distance().in(DISTANCE_STORE_UNIT));
        double[] datumArray = {
            datum.hoodAngle().in(HOOD_ANGLE_STORE_UNIT), datum.flywheelSpeed().in(FLYWHEEL_SPEED_STORE_UNIT)
        };

        var entry = table.getEntry(distanceStr);
        entry.setDoubleArray(datumArray);
        entry.setPersistent();
    }

    public List<Datum> getSortedData() {
        if (data.size() >= 2) {
            return new ArrayList<>(data.values());
        } else {
            return new ArrayList<>(estimateData.values());
        }
    }

    @Override
    public void accept(NetworkTable table, String key, NetworkTableEvent event) {
        assert event.is(Kind.kValueAll);

        double distance = Double.parseDouble(key);
        double[] datumArray = event.valueData.value.getDoubleArray();

        if (datumArray.length != 2) {
            DriverStation.reportWarning("Ignoring shooter datum [" + key + "] with invalid format", false);
            return;
        }

        Datum datum = new Datum(
                DISTANCE_STORE_UNIT.of(distance),
                HOOD_ANGLE_STORE_UNIT.of(datumArray[0]),
                FLYWHEEL_SPEED_STORE_UNIT.of(datumArray[1]));

        data.put(datum.distance(), datum);
    }
}
