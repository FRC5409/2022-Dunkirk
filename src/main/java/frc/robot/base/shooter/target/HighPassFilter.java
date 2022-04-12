package frc.robot.base.shooter.target;

import java.util.function.Supplier;

import frc.robot.utils.Vector2;

public class HighPassFilter extends FilterBase {
    private final Supplier<Double> kSmoothingFactor;
    private Vector2 kLastTarget;

    public HighPassFilter(Supplier<Double> kSmoothingFactor) {
        this.kSmoothingFactor = kSmoothingFactor;
        kLastTarget = null;
    }

    @Override
    public Vector2 update(Vector2 target) {
        if (kLastTarget == null) {
            kLastTarget = target;
        } else {
            double theta = kLastTarget.dot(target);
            double t = Math.pow((1 - theta)/2, kSmoothingFactor.get());

            double q = Math.sin(theta);

            double i = Math.sin((1-t)*theta) / q;
            double k = Math.sin(t*theta) / q;

            kLastTarget.x = kLastTarget.x * i + target.x * k;
            kLastTarget.y = kLastTarget.y * i + target.y * k;
        }

        return kLastTarget;
    }

    @Override
    public void reset() {
        kLastTarget = null;
    }
    
}
