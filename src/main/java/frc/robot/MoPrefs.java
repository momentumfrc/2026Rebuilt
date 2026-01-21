package frc.robot;

import edu.wpi.first.units.Units;
import frc.robot.molib.prefs.LinearVelocityUnitPref;
import frc.robot.molib.prefs.MoPrefsBase;

public class MoPrefs extends MoPrefsBase {
    public static LinearVelocityUnitPref examplePreference =
            metersPerSecPref("DUMMY PREF", Units.MetersPerSecond.of(5));
}
