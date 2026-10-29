package first.robot;

import first.robot.molib.prefs.AngularVelocityUnitPref;
import first.robot.molib.prefs.LinearVelocityUnitPref;
import first.robot.molib.prefs.MoPrefsBase;
import first.robot.molib.prefs.Pref;
import first.robot.molib.prefs.TimeUnitPref;
import first.robot.molib.prefs.UnitPref;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.Units;
import org.wpilib.units.VoltageUnit;

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

    public static final UnitPref<CurrentUnit> boostDriveMotorLimit =
            ampsPref("Swerve Boost Drive Limit", Units.Amps.of(60));
    public static final UnitPref<CurrentUnit> boostSteerMotorLimit =
            ampsPref("Swerve Boost Steer Limit", Units.Amps.of(25));

    public static final Pref<Double> driverRumble = unitlessDoublePref("Driver Rumble", 0.4);

    // **** SHOT BLOCKER ****
    public static final UnitPref<CurrentUnit> shotBlockerCurrentLimit =
            ampsPref("Shot Blocker Current Limit", Units.Amps.of(40));
    public static final Pref<Double> shotBlockerMaxHeight = unitlessDoublePref("Shot Blocker Max Height", 2000);
    public static final UnitPref<VoltageUnit> shotBlockerZeroPower =
            voltsPref("shotBlocker Zero Power", Units.Volts.of(-6));
    public static final TimeUnitPref shotBlockerSpinupTime =
            secondsPref("shotBlocker Spinup Time", Units.Seconds.of(0.5));
    public static final TimeUnitPref shotBlockerZeroTimeCutoff =
            secondsPref("shotBlocker Zero Time", Units.Seconds.of(0.25));
    public static final AngularVelocityUnitPref shotBlockerStallThreshold =
            rpmPref("shotBlocker Stall Threshold", Units.RPM.of(2));
    public static final Pref<Double> shotBlockerZeroPosition = unitlessDoublePref("shotBlocker Zero Pos.", -0.25);
    public static final UnitPref<VoltageUnit> shotBlockerMaxVoltage =
            voltsPref("Shot Blocker Max Voltage", Units.Volt.of(-1));
}
