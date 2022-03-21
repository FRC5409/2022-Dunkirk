package frc.robot.training;

import java.util.HashMap;
import java.util.Map;

import org.jetbrains.annotations.Nullable;

import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.ShooterExecutionModel;

public class TrainerContext {
    private Map<ShooterMode, ShooterExecutionModel> _models;
    private ShooterMode _mode;
    private Setpoint _target;
    private double _distance;

    public TrainerContext(Setpoint initialTarget) {
        _models = new HashMap<>();
        _target = initialTarget;
        _distance = 0.0;
    }

    public void setModel(ShooterExecutionModel model) {
        setModel(_mode, model);
    }

    public void setModel(ShooterMode mode, ShooterExecutionModel model) {
        _models.put(mode, model);
    }
    
    public void setSetpoint(Setpoint target) {
        _target = target;
    }
    
    public void setDistance(double distance) {
        _distance = distance;
    }

    public void setMode(ShooterMode mode) {
        _mode = mode;
    }

    @Nullable
    public ShooterExecutionModel getModel() {
        return _models.get(_mode);
    }
    
    @Nullable
    public ShooterExecutionModel getModel(ShooterMode mode) {
        return _models.get(mode);
    }

    public Setpoint getSetpoint() {
        return _target;
    }

    public double getDistance() {
        return _distance;
    }

    public ShooterMode getMode() {
        return _mode;
    }
}
