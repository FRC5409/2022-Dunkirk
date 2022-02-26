package frc.robot.base;

public interface Property<T> {
    T set(T value);

    T get();
}
