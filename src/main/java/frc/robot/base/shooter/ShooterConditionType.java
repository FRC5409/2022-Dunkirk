package frc.robot.base.shooter;

public enum ShooterConditionType {
    kNone(0x0),
    kFlywheelReached(0x01),
    kTurretReached(0x02),
    kIndexerArmed(0x04),
    
    kAll(kFlywheelReached.value |
         kTurretReached.value |
         kIndexerArmed.value);

    public final int value;

    private ShooterConditionType(int value) {
        this.value = value;
    }
}
