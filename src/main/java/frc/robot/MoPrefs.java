package frc.robot;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units;
import frc.robot.molib.prefs.AngularVelocityUnitPref;
import frc.robot.molib.prefs.LinearVelocityUnitPref;
import frc.robot.molib.prefs.MoPrefsBase;
import frc.robot.molib.prefs.Pref;
import frc.robot.molib.prefs.TimeUnitPref;
import frc.robot.molib.prefs.UnitPref;

public class MoPrefs extends MoPrefsBase {
    public static final LinearVelocityUnitPref swerveMaxAllowedSpeed =
            metersPerSecPref("Swerve Max Allowed Speed", Units.MetersPerSecond.of(5));
    public static final LinearVelocityUnitPref swerveMaxPossibleSpeed =
            metersPerSecPref("Swerve Max Possible Speed", Units.MetersPerSecond.of(5));
    public static final AngularVelocityUnitPref swerveMaxAllowedSpin =
            rotationsPerSecPref("Swerve Max Allowed Spin", Units.RotationsPerSecond.of(1));
    public static final AngularVelocityUnitPref swerveMaxPossibleSpin =
            rotationsPerSecPref("Swerve Max Possible Spin", Units.RotationsPerSecond.of(2));
    public static final AngularVelocityUnitPref kickerRunSpeed =
            rotationsPerSecPref("Kicker Run Speed", Units.RevolutionsPerSecond.of(1500));
    public static final AngularVelocityUnitPref indexerRunSpeed =
            rotationsPerSecPref("Indexer Run Speed", Units.RevolutionsPerSecond.of(1500));

    public static final TimeUnitPref limelightPoseRefreshDelay =
            secondsPref("Limelight Pose Refresh Delay", Units.Seconds.of(0.02));

    public static final Pref<Double> inputDeadband = unitlessDoublePref("Input DeadBand", 0.05);

    public static final PerUnit<DimensionlessUnit, AngleUnit> TicksPerRotation = Units.Value.per(Units.Revolution);

    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> kickerEncoderScale =
            encoderTicksPerRotationPref("Kicker Encoder Scale", TicksPerRotation.ofNative(32));

    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> indexerEncoderScale =
            encoderTicksPerRotationPref("Indexer Encoder Scale", TicksPerRotation.ofNative(32));
}
