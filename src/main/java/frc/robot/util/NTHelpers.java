package frc.robot.util;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.HashMap;
import java.util.Map;

public class NTHelpers {

    public static NetworkTable getTable(String name) {
        return NetworkTableInstance.getDefault().getTable(name);
    }

    /**
     * Return an entry representing a boolean value on the network tables, with an initial defaultValue set
     * on robot startup.
     * <p>
     * The intention behind this helper is to provide a shortcut for getting an entry and publishing an initial value.
     * This should for values that will be set on the dashboard and propagated to the robot.
     */
    public static BooleanEntry getBooleanEntry(NetworkTable table, String name, boolean defaultValue) {
        var entry = table.getBooleanTopic(name).getEntry(defaultValue);
        entry.set(defaultValue);
        return entry;
    }

    /**
     * Return an entry representing a double value on the network tables, with an initial defaultValue set
     * on robot startup.
     * <p>
     * The intention behind this helper is to provide a shortcut for getting an entry and publishing an initial value.
     * This should for values that will be set on the dashboard and propagated to the robot.
     */
    public static DoubleEntry getDoubleEntry(NetworkTable table, String name, double defaultValue) {
        var entry = table.getDoubleTopic(name).getEntry(defaultValue);
        entry.set(defaultValue);
        return entry;
    }

    public static <T extends Enum<?>> SendableChooser<T> enumToChooser(Class<T> toConvert) {
        return enumToChooser(toConvert, toConvert.getEnumConstants()[0]);
    }

    public static <T extends Enum<?>> SendableChooser<T> enumToChooser(Class<T> toConvert, T defaultValue) {
        var chooser = new SendableChooser<T>();
        chooser.setDefaultOption(defaultValue.name(), defaultValue);
        for (T entry : toConvert.getEnumConstants()) {
            if (entry != defaultValue) {
                chooser.addOption(entry.name(), entry);
            }
        }
        return chooser;
    }

    private static final Map<String, Sendable> tablesToData = new HashMap<>();

    public static void publishSendable(NetworkTable table, Sendable data) {
        String name = SendableRegistry.getName(data);
        if (!name.isEmpty()) {
            publishSendable(table, name, data);
        }
    }

    public static void publishSendable(NetworkTable table, String key, Sendable data) {
        NetworkTable dataTable = table.getSubTable(key);
        if (tablesToData.get(key) == data) {
            return;
        }
        tablesToData.put(key, data);
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(dataTable);
        SendableRegistry.publish(data, builder);
        builder.startListeners();
        dataTable.getEntry(".name").setString(key);
    }

    public static void updateSendables() {
        for (Sendable data : tablesToData.values()) {
            SendableRegistry.update(data);
        }
    }

    private NTHelpers() {
        throw new UnsupportedOperationException("Cannot instantiate static utility class [NTHelpers]");
    }
}
