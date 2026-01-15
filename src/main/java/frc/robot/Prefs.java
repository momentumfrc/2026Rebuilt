package frc.robot;

import edu.wpi.first.units.Units;
import frc.robot.molib.prefs.LinearVelocityUnitPref;
import frc.robot.molib.prefs.MoPrefs;

public class Prefs extends MoPrefs {
    public static LinearVelocityUnitPref examplePreference =
            metersPerSecPref("DUMMY PREF", Units.MetersPerSecond.of(5));
}
