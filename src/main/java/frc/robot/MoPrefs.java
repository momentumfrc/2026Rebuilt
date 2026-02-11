package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.molib.prefs.AngularVelocityUnitPref;
import frc.robot.molib.prefs.DimensionlessUnitPref;
import frc.robot.molib.prefs.LinearVelocityUnitPref;
import frc.robot.molib.prefs.MoPrefsBase;
import frc.robot.molib.prefs.Pref;

public class MoPrefs extends MoPrefsBase {
    public static final LinearVelocityUnitPref swerveMaxAllowedSpeed =
            metersPerSecPref("Swerve Max Allowed Speed", Units.MetersPerSecond.of(5));
    public static final LinearVelocityUnitPref swerveMaxPossibleSpeed =
            metersPerSecPref("Swerve Max Possible Speed", Units.MetersPerSecond.of(5));
    public static final AngularVelocityUnitPref swerveMaxAllowedSpin =
            rotationsPerSecPref("Swerve Max Allowed Spin", Units.RotationsPerSecond.of(1));
    public static final AngularVelocityUnitPref swerveMaxPossibleSpin =
            rotationsPerSecPref("Swerve Max Possible Spin", Units.RotationsPerSecond.of(2));
    public static final DimensionlessUnitPref kickerRunPercentage = percentPref("Kicker Run Percentage", Units.Percent.of(0.6));
    public static final DimensionlessUnitPref indexerRunPercentage = percentPref("Indexer Run Percentage", Units.Percent.of(0.6));

    public static final Pref<Double> inputDeadband = unitlessDoublePref("Input DeadBand", 0.05);
}
