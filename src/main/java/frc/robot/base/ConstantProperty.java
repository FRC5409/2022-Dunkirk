package frc.robot.base;

public class ConstantProperty<T> implements Property<T> {
    protected final T m_value;

    public ConstantProperty() {
        m_value = null;
    }

    public ConstantProperty(T value) {
        m_value = value;
    }

    public ConstantProperty(Property<T> other) {
        m_value = other.get();
    }

    @Override
    public T set(T value) {
        return m_value;
    }

    @Override
    public T get() {
        return m_value;
    } 
}
