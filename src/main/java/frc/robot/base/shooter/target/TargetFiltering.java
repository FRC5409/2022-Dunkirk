package frc.robot.base.shooter.target;

import java.util.function.Supplier;

public final class TargetFiltering {
    public static FilterFactory<HighPassFilter> highPass(double kSmoothingFactor) {
        return () -> new HighPassFilter(() -> kSmoothingFactor);
    }

    public static FilterFactory<HighPassFilter> highPass(Supplier<Double> kSmoothingFactor) {
        return () -> new HighPassFilter(kSmoothingFactor);
    }

    public static FilterFactory<FilterBase> none() {
        return NullFilter::new;
    }
}
