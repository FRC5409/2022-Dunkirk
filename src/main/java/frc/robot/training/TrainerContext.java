package frc.robot.training;

import java.util.HashMap;
import java.util.Map;

import org.jetbrains.annotations.Nullable;

import frc.robot.base.Model4;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.odometry.ShooterOdometryModel;

public class TrainerContext {
    private Map<ShooterMode, Model4> executionModels;
    private Map<ShooterMode, ShooterOdometryModel> odometryModels;
    private ShooterMode mode;
    private Setpoint target;
    private double distance;

    public TrainerContext(Setpoint initialTarget) {
        executionModels = new HashMap<>();
        odometryModels = new HashMap<>();
        target = initialTarget;
        distance = 0.0;
    }

    public void setExecutionModel(Model4 model) {
        setExecutionModel(mode, model);
    }

    public void setExecutionModel(ShooterMode mode, Model4 model) {
        executionModels.put(mode, model);
    }

    public void setOdometryModel(ShooterOdometryModel model) {
        setOdometryModel(mode, model);
    }

    public void setOdometryModel(ShooterMode mode, ShooterOdometryModel model) {
        odometryModels.put(mode, model);
    }
    
    public void setSetpoint(Setpoint target) {
        this.target = target;
    }
    
    public void setDistance(double distance) {
        this.distance = distance;
    }

    public void setMode(ShooterMode mode) {
        this.mode = mode;
    }

    @Nullable
    public Model4 getExecutionModel() {
        return executionModels.get(mode);
    }
    
    @Nullable
    public Model4 getExecutionModel(ShooterMode mode) {
        return executionModels.get(mode);
    }
    
    @Nullable
    public ShooterOdometryModel getOdometryModel() {
        return odometryModels.get(mode);
    }
    
    @Nullable
    public ShooterOdometryModel getOdometryModel(ShooterMode mode) {
        return odometryModels.get(mode);
    }

    public Setpoint getSetpoint() {
        return target;
    }

    public double getDistance() {
        return distance;
    }

    public ShooterMode getMode() {
        return mode;
    }
}
