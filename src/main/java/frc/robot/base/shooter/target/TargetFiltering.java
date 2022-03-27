package frc.robot.base.shooter.target;

import java.util.function.Supplier;

public final class TargetFiltering {
    public static HighPassFilter highPass(Supplier<Double> kSmoothingFactor) {
        return new HighPassFilter(kSmoothingFactor);
    }
}
