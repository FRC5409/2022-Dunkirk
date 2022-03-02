package frc.robot.training;

import frc.robot.utils.Range;
import org.jetbrains.annotations.Nullable;

public class Setpoint {
    private final SetpointType _type;
    private final Setpoint _parent;
    private final double _target;
    private final Range _range;

    public Setpoint(double target, Range range) {
        this(null, target, range, SetpointType.kRoot);
    }

    protected Setpoint(double target, Range range, SetpointType type) {
        this(null, target, range, type);
    }

    public Setpoint(@Nullable Setpoint parent, double target, Range range, SetpointType type) {
        if (!range.contains(target))
            throw new IllegalArgumentException("Target cannot exceed range");

        _parent = parent;
        _target = target;
        _range = range;
        _type = type;
    }
    
    public Setpoint branch(SetpointType type) {
        switch (type) {
            case kLeft: {
                Range range = new Range(_range.min(), _target);
                return new Setpoint(this, range.mid(), range, SetpointType.kLeft);
            } 
            case kRight: {
                Range range = new Range(_target, _range.max());
                return new Setpoint(this, range.mid(), range, SetpointType.kRight);
            }
            case kCenter: {
                Range range = new Range(
                    (_target + _range.min()) / 2,  (_target + _range.max()) / 2
                );
                return new Setpoint(this, _target, range, SetpointType.kCenter);
            }
            
            default: return this;
        }
    }
    
    @Nullable
    public Setpoint getParent() {
        return _parent;
    }

    public Range getRange() {
        return _range;
    }

    public double getTarget() {
        return _target;
    }

    public SetpointType getType() {
        return _type;
    }
}
