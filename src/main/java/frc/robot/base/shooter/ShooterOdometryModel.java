package frc.robot.base.shooter;

import frc.robot.utils.Vector2;

public class ShooterOdometryModel {
    public final double kPitch;
    public final double kHeight;
    public final double kOffset;
    public final Vector2 kFieldOfView;
    
    public ShooterOdometryModel(
        double kPitch,
        double kHeight,
        double kOffset,
        Vector2 kFieldOfView
    ) {
        this.kPitch = kPitch;
        this.kHeight = kHeight;
        this.kOffset = kOffset;
        this.kFieldOfView = kFieldOfView;
    }
}
