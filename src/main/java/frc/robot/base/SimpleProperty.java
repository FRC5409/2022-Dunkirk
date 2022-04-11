package frc.robot.base;

import java.util.function.Consumer;
import java.util.function.Supplier;

final class SimpleProperty<T> implements Property<T> {
    private final Supplier<T> getter;
    private final Consumer<T> setter;

    public SimpleProperty(Consumer<T> setter, Supplier<T> getter) {
        this.setter = setter;
        this.getter = getter;
    }

    @Override
    public T set(T value) {
        final T last = getter.get();
        setter.accept(value);
        return last;
    }

    @Override
    public T get() {
        return getter.get();
    }
}
