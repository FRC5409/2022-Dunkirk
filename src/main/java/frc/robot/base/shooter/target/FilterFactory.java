package frc.robot.base.shooter.target;

public interface FilterFactory<T extends FilterBase> {
    public abstract FilterBase create();
}
