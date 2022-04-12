package frc.robot.base.shooter.target;

import frc.robot.utils.Vector2;

public abstract class FilterBase {
    public abstract Vector2 update(Vector2 target);
    public abstract void reset();
}
