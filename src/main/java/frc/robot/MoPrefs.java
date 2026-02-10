package frc.robot;

import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
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

    // Intake prefereces
    public static final UnitPref<VoltageUnit> intakeRollerVoltage = voltsPref("Intake Roller Power", Units.Volts.of(5));

    public static final UnitPref<VoltageUnit> intakeWristVoltage = voltsPref("Intake Wrist Power", Units.Volts.of(5));
    public static final UnitPref<VoltageUnit> intakeWristHoldVoltage =
            voltsPref("Intake Wrist Hold Power", Units.Volts.of(1));

    public static final UnitPref<CurrentUnit> intakeWristSmartCurrentLimit =
            ampsPref("Intake Wrist Smart Current Limit", Units.Amps.of(20));

    public static final TimeUnitPref intakeCurrentWristTime =
            secondsPref("Intake Current Wrist Time", Units.Seconds.one());

    public static final Pref<Double> inputDeadband = unitlessDoublePref("Input DeadBand", 0.05);
}
