package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.SparkMaxPIDController;

public final class MotorUtils {
    public static final WPI_TalonFX setGains(WPI_TalonFX motor, int slotIdx, Gains gains) {
        motor.config_kP(slotIdx, gains.kP);
        motor.config_kI(slotIdx, gains.kI);
        motor.config_kD(slotIdx, gains.kD);
        motor.config_kF(slotIdx, gains.kF);

        return motor;
    }

    public static final SparkMaxPIDController setGains(SparkMaxPIDController controller, Gains gains) {
        controller.setP(gains.kP);
        controller.setI(gains.kI);
        controller.setD(gains.kD);
        controller.setFF(gains.kF);

        return controller;
    }
}
