package frc.robot.training;

import java.util.HashMap;
import java.util.Map;

import org.jetbrains.annotations.Nullable;

import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.ShooterModel;

public class TrainerContext {
    private Map<ShooterMode, ShooterModel> _models;
    private ShooterMode _mode;
    private Setpoint _target;
    private double _distance;

    public TrainerContext(Setpoint initialTarget) {
        _models = new HashMap<>();
        _target = initialTarget;
        _distance = 0.0;
    }

    public void setModel(ShooterModel model) {
        setModel(getMode(), model);
    }

    public void setModel(ShooterMode mode, ShooterModel model) {
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
    public ShooterModel getModel() {
        return _models.get(getMode());
    }
    
    @Nullable
    public ShooterModel getModel(ShooterMode mode) {
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
