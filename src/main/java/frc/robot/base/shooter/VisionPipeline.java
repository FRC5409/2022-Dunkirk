package frc.robot.base.shooter;

public enum VisionPipeline {
    NEAR_TARGETING(1),
    FAR_TARGETING(2);

    private final int _id;

    private VisionPipeline(int id) {
        _id = id;
    }

    public int id() {
        return _id;
    }
}
