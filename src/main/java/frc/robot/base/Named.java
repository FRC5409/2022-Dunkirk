package frc.robot.base;

public interface Named {
    static Named of(String name) {
        return new SimpleName(name);
    }

    String getName();
}
