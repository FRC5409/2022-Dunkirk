package frc.robot.base.lights;

public enum AnimationID {
    Default((byte)0b00000000),
    One((byte)0b00000001),
    Two((byte)0b00000010),
    Three((byte)0b00000011);

    private final byte idx;

    private AnimationID(byte idx) {
        this.idx = idx;
    }

    public byte getId() {
        return idx;
    }
}
