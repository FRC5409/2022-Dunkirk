package frc.robot.base;

import java.util.function.Consumer;
import java.util.function.Supplier;

public interface Property<T> {
    public static <T> Property<T> of(Consumer<T> setter, Supplier<T> getter) {
        return new SimpleProperty<>(setter, getter);
    }
    T set(T value);

    T get();
}
