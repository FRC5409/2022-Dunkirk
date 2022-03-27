package frc.robot.base;

public class ValueProperty<T> implements Property<T> {
    protected T m_value;

    public ValueProperty() {
        m_value = null;
    }

    public ValueProperty(T value) {
        m_value = value;
    }

    public ValueProperty(Property<T> other) {
        m_value = other.get();
    }

    @Override
    public T set(T value) {
        final T last = m_value;
        m_value = value;
        return last;
    }

    @Override
    public T get() {
        return m_value;
    } 
}
