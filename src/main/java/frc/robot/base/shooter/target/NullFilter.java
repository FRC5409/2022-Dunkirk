package frc.robot.base.shooter.target;

import frc.robot.utils.Vector2;

public class NullFilter extends FilterBase {
    @Override
    public Vector2 update(Vector2 target) {
        return target;
    }

    @Override
    public void reset() {
    }
}
