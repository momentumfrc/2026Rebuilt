package frc.robot;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units;
import frc.robot.molib.MoUnits;
import frc.robot.molib.prefs.AngleUnitPref;
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

    public static final TimeUnitPref limelightPoseRefreshDelay =
            secondsPref("Limelight Pose Refresh Delay", Units.Seconds.of(0.02));

    public static final Pref<Double> inputDeadband = unitlessDoublePref("Input DeadBand", 0.05);

    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> turretRelativeEncoderScale =
            encoderTicksPerRotationPref(
                    "Turret Relative Encoder Scale", MoUnits.EncoderTicksPerRotation.ofNative(255.0 / 8.0));
    public static final AngleUnitPref turretMinSoftLimit = degreesPref("Turret Min Soft Limit", Units.Degrees.of(5));
    public static final AngleUnitPref turretMaxSoftLimit = degreesPref("Turret Max Soft Limit", Units.Degrees.of(355));

    public static final AngleUnitPref turretEncoder1Zero =
            rotationsPref("Turret Encoder 1 Zero", Units.Rotations.zero());
    public static final AngleUnitPref turretEncoder2Zero =
            rotationsPref("Turret Encoder 2 Zero", Units.Rotations.zero());
}
