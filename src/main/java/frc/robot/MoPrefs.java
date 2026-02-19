package frc.robot;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
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
    public static final AngularVelocityUnitPref kickerRunSpeed =
            rotationsPerSecPref("Kicker Run Speed", Units.RevolutionsPerSecond.of(1500));
    public static final AngularVelocityUnitPref indexerRunSpeed =
            rotationsPerSecPref("Indexer Run Speed", Units.RevolutionsPerSecond.of(1500));

    // Intake prefereces
    public static final UnitPref<VoltageUnit> intakeRollerVoltage = voltsPref("Intake Roller Power", Units.Volts.of(5));

    public static final UnitPref<VoltageUnit> intakeWristVoltage = voltsPref("Intake Wrist Power", Units.Volts.of(5));
    public static final UnitPref<VoltageUnit> intakeWristHoldVoltage =
            voltsPref("Intake Wrist Hold Power", Units.Volts.of(1));

    public static final UnitPref<CurrentUnit> intakeWristSmartCurrentLimit =
            ampsPref("Intake Wrist Smart Current Limit", Units.Amps.of(20));

    public static final TimeUnitPref intakeHighCurrentWristTime =
            secondsPref("Intake High Current Wrist Time", Units.Seconds.one());

    public static final TimeUnitPref limelightPoseRefreshDelay =
            secondsPref("Limelight Pose Refresh Delay", Units.Seconds.of(0.02));

    public static final Pref<Double> inputDeadband = unitlessDoublePref("Input DeadBand", 0.05);

    public static final PerUnit<DimensionlessUnit, AngleUnit> TicksPerRotation = Units.Value.per(Units.Revolution);

    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> hoodEncoderScale =
            encoderTicksPerRotationPref("Hood Encoder Scale", TicksPerRotation.ofNative(32));

    public static final AngleUnitPref hoodDeadzonePosition = degreesPref("Hood Deadzone Position", Units.Degrees.of(0));
    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> kickerEncoderScale =
            encoderTicksPerRotationPref("Kicker Encoder Scale", TicksPerRotation.ofNative(32));

    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> indexerEncoderScale =
            encoderTicksPerRotationPref("Indexer Encoder Scale", TicksPerRotation.ofNative(32));
    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> turretRelativeEncoderScale =
            encoderTicksPerRotationPref(
                    "Turret Relative Encoder Scale", MoUnits.EncoderTicksPerRotation.ofNative(255.0 / 8.0));

    /**
     * This value is summed with the absolute encoder reading, and the result is used to set the relative encoder.
     * Theoretically, the absolute zero is 15 degrees counter-clockwise of the relative zero, so the theoretical value
     * is 15 degrees. To calibrate this, first calibrate the encoder zero. Then manually move the turret to be aligned
     * with the front of the robot. This position should be 360deg on the relative encoder, so subtract
     * (360deg - abs_encoder_reading), and set the result as the relative encoder offset.
     */
    public static final AngleUnitPref turretRelativeEncoderOffset =
            rotationsPref("Turret Relative Encoder Offset", Units.Rotations.of(15));

    public static final AngleUnitPref turretMinSoftLimit = degreesPref("Turret Min Soft Limit", Units.Degrees.of(17.5));
    public static final AngleUnitPref turretMaxSoftLimit =
            degreesPref("Turret Max Soft Limit", Units.Degrees.of(362.5));

    public static final UnitPref<VoltageUnit> turretMaxPower = voltsPref("Turret Max Power", Units.Volts.of(6));
    public static final TimeUnitPref turretVoltRampRate = secondsPref("Turret Voltage Ramp Rate", Units.Seconds.of(0));

    public static final AngularVelocityUnitPref flywheelSpeedTolerance =
            rotationsPerSecPref("Flywheel Speed Tolerance", Units.RotationsPerSecond.of(20));

    public static final AngleUnitPref turretEncoder1Zero =
            rotationsPref("Turret Encoder 1 Zero", Units.Rotations.zero());
    public static final AngleUnitPref turretEncoder2Zero =
            rotationsPref("Turret Encoder 2 Zero", Units.Rotations.zero());
}
