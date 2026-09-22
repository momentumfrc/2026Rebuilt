package first.robot.util;

import first.robot.molib.motune.MoTuner;

public class MutablePIDConstants {
    public double kP, kI, kD, kIZone;

    public MoTuner.Builder getTuner(String name) {
        return MoTuner.builder(name)
                .p(p -> {
                    this.kP = p;
                })
                .i(i -> {
                    this.kI = i;
                })
                .d(d -> {
                    this.kD = d;
                })
                .iZone(i -> {
                    this.kIZone = i;
                });
    }
}
