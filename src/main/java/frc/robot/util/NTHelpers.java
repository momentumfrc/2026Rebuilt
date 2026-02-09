package frc.robot.util;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanSubscriber;
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

    public static BooleanChangeSubscriber getBooleanChangeSubscriber(
            NetworkTable table, String name, boolean defaultValue) {
        var entry = table.getBooleanTopic(name).getEntry(defaultValue);
        entry.set(defaultValue);
        return new BooleanChangeSubscriber(entry, defaultValue);
    }

    /** Wraps a {@link BooleanSubscriber} to indicate whether the value has changed since the last time it was retrieved. */
    public static class BooleanChangeSubscriber {
        private final BooleanSubscriber subscriber;

        public static enum Value {
            TRUE,
            FALSE,
            NO_CHANGE
        };

        private Value lastValue;

        public BooleanChangeSubscriber(BooleanSubscriber subscriber) {
            this.subscriber = subscriber;
            this.lastValue = null;
        }

        public BooleanChangeSubscriber(BooleanSubscriber subscriber, boolean defaultValue) {
            this.subscriber = subscriber;
            this.lastValue = defaultValue ? Value.TRUE : Value.FALSE;
        }

        public BooleanChangeSubscriber withNotifyImmediately() {
            this.lastValue = null;
            return this;
        }

        public Value get() {
            Value value = subscriber.get() ? Value.TRUE : Value.FALSE;
            if (lastValue == null || lastValue != value) {
                lastValue = value;
                return value;
            } else {
                return Value.NO_CHANGE;
            }
        }
    }

    private NTHelpers() {
        throw new UnsupportedOperationException("Cannot instantiate static utility class [NTHelpers]");
    }
}
