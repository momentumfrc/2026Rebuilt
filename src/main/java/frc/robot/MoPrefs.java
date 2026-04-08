package frc.robot;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.molib.MoUnits;
import frc.robot.molib.prefs.AngleUnitPref;
import frc.robot.molib.prefs.AngularAccelerationUnitPref;
import frc.robot.molib.prefs.AngularVelocityUnitPref;
import frc.robot.molib.prefs.DimensionlessUnitPref;
import frc.robot.molib.prefs.LinearVelocityUnitPref;
import frc.robot.molib.prefs.MoPrefsBase;
import frc.robot.molib.prefs.Pref;
import frc.robot.molib.prefs.TimeUnitPref;
import frc.robot.molib.prefs.UnitPref;

public class MoPrefs extends MoPrefsBase {
    // **** SWERVE DRIVE ****
    public static final LinearVelocityUnitPref swerveMaxAllowedSpeed =
            metersPerSecPref("Swerve Max Allowed Speed", Units.MetersPerSecond.of(5));
    public static final LinearVelocityUnitPref swerveMaxPossibleSpeed =
            metersPerSecPref("Swerve Max Possible Speed", Units.MetersPerSecond.of(5));
    public static final AngularVelocityUnitPref swerveMaxAllowedSpin =
            rotationsPerSecPref("Swerve Max Allowed Spin", Units.RotationsPerSecond.of(1));
    public static final AngularVelocityUnitPref swerveMaxPossibleSpin =
            rotationsPerSecPref("Swerve Max Possible Spin", Units.RotationsPerSecond.of(2));
    public static final Pref<Double> inputDeadband = unitlessDoublePref("Swerve Input Deadband", 0.05);
    public static final Pref<Boolean> inputTranslationCubed = booleanPref("Swerve Input Translation Cubed?", true);
    public static final Pref<Boolean> inputRotationCubed = booleanPref("Swerve Input Rotation Cubed?", true);
    public static final Pref<Boolean> enableHeadingCorrection = booleanPref("Swerve heading correction", true);

    public static final Pref<Double> driveIntakingSlowSpeed = unitlessDoublePref("Swerve intaking slow speed", 0.4);

    public static final UnitPref<CurrentUnit> boostDriveMotorLimit =
            ampsPref("Swerve Boost Drive Limit", Units.Amps.of(60));
    public static final UnitPref<CurrentUnit> boostSteerMotorLimit =
            ampsPref("Swerve Boost Steer Limit", Units.Amps.of(25));

    // **** INTAKE WRIST ****

    public static final UnitPref<CurrentUnit> intakeWristSmartCurrentLimit =
            ampsPref("Intake Wrist Smart Current Limit", Units.Amps.of(20));

    public static final TimeUnitPref intakeRampTime = secondsPref("Intake Ramp Time", Units.Seconds.of(0.5));

    public static final TimeUnitPref intakeWristMoveTimeout =
            secondsPref("Intake Wrist Move Timeout", Units.Seconds.of(5));

    public static final AngleUnitPref intakeWristDeployPosition =
            rotationsPref("Intake Wrist Deploy Position", Units.Rotations.of(0.5));
    public static final AngleUnitPref intakeWristRetractPosition =
            rotationsPref("Intake Wrist Retract Position", Units.Rotations.of(0.05));

    public static final TimeUnitPref intakeWristAgitatePeriod =
            secondsPref("Intake Wrist Agitate Period", Units.Seconds.of(5));

    public static final UnitPref<VoltageUnit> intakeWristZeroVoltage =
            voltsPref("Intake Wrist Zero Voltage", Units.Volts.of(8));
    public static final UnitPref<CurrentUnit> intakeWristZeroThresh =
            ampsPref("Intake Wrist Zero Current Thresh", Units.Amps.of(18));
    public static final TimeUnitPref intakeWristZeroTime = secondsPref("Intake Wrist Zero Time", Units.Seconds.of(0.5));

    public static final UnitPref<VoltageUnit> intakeWristFwdHoldVoltage =
            voltsPref("Intake Wrist Fwd Hold Power", Units.Volts.of(0.125));
    public static final UnitPref<VoltageUnit> intakeWristRevHoldVoltage =
            voltsPref("Intake Wrist Rev Hold Power", Units.Volts.of(0.125));
    public static final Pref<Boolean> intakeWristFwdHoldApplyFF = booleanPref("Intake Wrist Fwd Apply FF?", true);

    public static final AngularVelocityUnitPref intakeWristMaxVelocity =
            degreesPerSecPref("Intake Wrist Max Velocity", Units.DegreesPerSecond.of(360));
    public static final AngularAccelerationUnitPref intakeWristMaxAccel =
            degreesPerSec2Pref("Intake Wrist Max Acceleration", Units.DegreesPerSecondPerSecond.of(90));

    // **** INTAKE ROLLERS ****
    public static final UnitPref<VoltageUnit> intakeRollerVoltage = voltsPref("Intake Roller Power", Units.Volts.of(5));
    public static final UnitPref<CurrentUnit> intakeRollerSmartCurrentLimit =
            ampsPref("Intake Rollers Current Limit", Units.Amps.of(20));

    // **** HOPPER / INDEXER ****
    public static final UnitPref<VoltageUnit> indexerRunPower = voltsPref("Indexer Run Power", Units.Volts.of(12));
    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> indexerEncoderScale =
            encoderTicksPerRotationPref("Indexer Encoder Scale", MoUnits.EncoderTicksPerRotation.ofNative(4));
    public static final UnitPref<CurrentUnit> indexerRollerSmartCurrentLimit =
            ampsPref("Indexer Current Limit", Units.Amps.of(40));

    public static final UnitPref<VoltageUnit> centeringRunPower = voltsPref("Centering Run Power", Units.Volts.of(8));
    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> centeringEncoderScale =
            encoderTicksPerRotationPref("Centering Encoder Scale", MoUnits.EncoderTicksPerRotation.ofNative(27 / 16));
    public static final UnitPref<CurrentUnit> CenteringSmartCurrentLimit =
            ampsPref("Centering Current Limit", Units.Amps.of(40));

    // **** KICKER ****
    public static final AngularVelocityUnitPref kickerRunSpeed = rpmPref("Kicker Run Speed", Units.RPM.of(1000));
    public static final UnitPref<CurrentUnit> kickerCurrentLimit = ampsPref("Kicker Current Limit", Units.Amps.of(40));
    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> kickerEncoderScale =
            encoderTicksPerRotationPref("Kicker Encoder Scale", MoUnits.EncoderTicksPerRotation.ofNative(4));
    public static final AngularVelocityUnitPref kickerClearSpeed = rpmPref("Kicker Clear Speed", Units.RPM.of(300));

    // **** TURRET ****
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
            degreesPref("Turret Relative Encoder Offset", Units.Degrees.of(15));

    public static final AngleUnitPref turretEncoder1Zero =
            rotationsPref("Turret Abs. Encoder 1 Zero", Units.Rotations.zero());
    public static final AngleUnitPref turretEncoder2Zero =
            rotationsPref("Turret Abs. Encoder 2 Zero", Units.Rotations.zero());

    public static final AngleUnitPref turretMinSoftLimit = degreesPref("Turret Min Soft Limit", Units.Degrees.of(17.5));
    public static final AngleUnitPref turretMaxSoftLimit =
            degreesPref("Turret Max Soft Limit", Units.Degrees.of(362.5));

    /**
     * The maximum output voltage to the turret motor. This value allows rough control of the turret's max speed, however
     * it is preferred to use turretMaxVelocity for more fine control.
     */
    public static final UnitPref<VoltageUnit> turretMaxPower = voltsPref("Turret Max Power", Units.Volts.of(12));
    /**
     * The minimum duration for the turret motor output voltage to sweep between 0 and 12 volts. This value is allows rough control
     * of the turret's max acceleration, however it is preferred to use turretMaxAcceleration for more fine control.
     */
    public static final TimeUnitPref turretVoltRampRate = secondsPref("Turret Voltage Ramp Rate", Units.Seconds.of(0));

    /**
     * The average duration between when a setpoint is commanded and when it is achieved for the turret and shooter subsystems.
     */
    public static final TimeUnitPref turretPhaseDelay = secondsPref("Turret Phase Delay", Units.Seconds.of(0.03));

    public static final AngularVelocityUnitPref turretMaxVelocity =
            degreesPerSecPref("Turret Max Velocity", Units.DegreesPerSecond.of(180));
    public static final AngularAccelerationUnitPref turretMaxAcceleration =
            degreesPerSec2Pref("Turret Max Acceleration", Units.DegreesPerSecondPerSecond.of(360));

    public static final AngleUnitPref turretFallbackSetpoint =
            degreesPref("Turret Fallback Setpoint", Units.Degrees.of(180));

    // **** SHOOTER HOOD ****
    public static final UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> hoodEncoderScale =
            encoderTicksPerRotationPref("Hood Encoder Scale", MoUnits.EncoderTicksPerRotation.ofNative(32));
    public static final AngularVelocityUnitPref hoodMaxVelocity =
            degreesPerSecPref("Hood Max Velocity", Units.DegreesPerSecond.of(1000));
    public static final AngularAccelerationUnitPref hoodMaxAcceleration =
            degreesPerSec2Pref("Hood Max Acceleration", Units.DegreesPerSecondPerSecond.of(1000));
    public static final AngleUnitPref hoodMinSoftLimit = degreesPref("Hood Min Soft Limit", Units.Degree.of(1));
    public static final AngleUnitPref hoodMaxSoftLimit = degreesPref("Hood Max Soft Limit", Units.Degrees.of(40));
    public static final UnitPref<VoltageUnit> hoodZeroPower = voltsPref("Hood Zero Power", Units.Volts.of(3));
    public static final UnitPref<CurrentUnit> hoodZeroCurrentThreshold =
            ampsPref("Hood Zero Current Threshold", Units.Amps.of(30));
    public static final TimeUnitPref hoodZeroTime = secondsPref("Hood Zero Time", Units.Seconds.of(0.5));
    public static final UnitPref<CurrentUnit> hoodCurrentLimit = ampsPref("Hood Current Limit", Units.Amps.of(40));

    public static final AngleUnitPref hoodDeadzonePosition = degreesPref("Hood Deadzone Position", Units.Degrees.of(0));

    public static final AngleUnitPref hoodFallbackSetpoint =
            degreesPref("Hood Fallback Setpoint", Units.Degrees.of(20));

    // **** SHOOTER FLYWHEEL ****
    public static final UnitPref<CurrentUnit> flywheelCurrentLimit =
            ampsPref("Flywheel Current Limit", Units.Amps.of(40));

    public static final AngularVelocityUnitPref flywheelFallbackSetpoint =
            rpmPref("Flywheel Fallback Setpoint", Units.RPM.of(3000));

    public static final AngularVelocityUnitPref flywheelClearSpeed = rpmPref("Flywheel Clear Speed", Units.RPM.of(300));
    public static final TimeUnitPref shooterClearTime = secondsPref("Shooter Clear Time", Units.Seconds.of(0.25));

    // **** LEDS ****
    public static final DimensionlessUnitPref ledBrightness = percentPref("LED Brightness", Units.Percent.of(40));

    // **** AUTO ****
    // TODO: Find auto preferences
    public static final TimeUnitPref autoOutpostWaitTime = secondsPref("Auto Outpost wait Time", Units.Seconds.of(2));

    public static final TimeUnitPref autoIntakeRunTime = secondsPref("Auto Intake Run Time", Units.Seconds.of(2));

    public static final TimeUnitPref autoShooterRunTime = secondsPref("Auto Shooter Run Time", Units.Seconds.of(3));
}
