package frc.robot.base;

import java.util.function.Consumer;
import java.util.function.Supplier;

public interface Property<T> {
    public static <T> Property<T> of(Consumer<T> setter, Supplier<T> getter) {
        return new SimpleProperty<>(setter, getter);
    }

    @SuppressWarnings("unchecked")
    public static <T, V> Property<V> cast(Property<T> property) {
        return new SimpleProperty<>(
             x -> property.set((T) x),
            () -> { return (V) property.get(); }
        );
    }

    T set(T value);

    T get();

    default boolean isEqual(T value) {
        return get().equals(value);
    };
}
