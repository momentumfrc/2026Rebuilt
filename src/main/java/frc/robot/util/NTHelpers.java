package frc.robot.util;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTHelpers {

    public static NetworkTable getTable(String name) {
        return NetworkTableInstance.getDefault().getTable(name);
    }

    public static BooleanEntry getBooleanEntry(NetworkTable table, String name, boolean defaultValue) {
        var entry = table.getBooleanTopic(name).getEntry(defaultValue);
        entry.set(defaultValue);
        return entry;
    }

    private NTHelpers() {
        throw new UnsupportedOperationException("Cannot instantiate static utility class [NTHelpers]");
    }
}
