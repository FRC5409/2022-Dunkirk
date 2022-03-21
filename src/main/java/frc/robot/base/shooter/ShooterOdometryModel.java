package frc.robot.base.shooter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utils.Vector2;

public class ShooterOdometryModel implements Sendable {
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("kPitch", () -> kPitch, null); 
        builder.addDoubleProperty("kHeight", () -> kHeight, null); 
        builder.addDoubleProperty("kOffset", () -> kOffset, null); 
        builder.addStringProperty("kFieldOfView", () -> kFieldOfView.toString(), null);
    }
}
