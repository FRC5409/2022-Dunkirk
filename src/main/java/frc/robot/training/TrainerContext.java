package frc.robot.training;

import frc.robot.utils.ShooterModel;

public class TrainerContext {
    private ShooterModel _model;
    private Setpoint _target;
    private double _distance;

    public TrainerContext(Setpoint initialTarget, ShooterModel model) {
        _model = model;
        _target = initialTarget;
        _distance = 0.0;
    }

    public void setModel(ShooterModel model) {
        _model = model;
    }
    
    public void setSetpoint(Setpoint target) {
        _target = target;
    }
    
    public void setDistance(double distance) {
        _distance = distance;
    }

    public ShooterModel getModel() {
        return _model;
    }

    public Setpoint getSetpoint() {
        return _target;
    }

    public double getDistance() {
        return _distance;
    }
}
