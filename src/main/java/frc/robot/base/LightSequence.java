package frc.robot.base;

public enum LightSequence {
    kDefault(0);

    private final int idx;

    private LightSequence(int idx) {
        this.idx = idx;
    }

    public int getId() {
        return idx;
    }
}
