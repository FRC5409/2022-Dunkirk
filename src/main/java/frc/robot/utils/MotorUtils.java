package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;

public final class MotorUtils {
    public static final WPI_TalonFX setGains(WPI_TalonFX motor, int slotIdx, Gains gains) {
        motor.config_kP(slotIdx, gains.kP);
        motor.config_kI(slotIdx, gains.kI);
        motor.config_kD(slotIdx, gains.kD);
        motor.config_kF(slotIdx, gains.kF);

        return motor;
    }

    public static final WPI_TalonFX lowerFollowerStatusPeriod(WPI_TalonFX motor) {
        motor.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

        return motor;
    }

    public static final WPI_TalonFX setDefaultStatusPeriod(WPI_TalonFX motor) {
        motor.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

        return motor;
    }

    public static final WPI_TalonFX lowerNonImportantPeriods(WPI_TalonFX motor) {
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        
        return motor;
    }

    public static final SparkMaxPIDController setGains(SparkMaxPIDController controller, Gains gains) {
        controller.setP(gains.kP);
        controller.setI(gains.kI);
        controller.setD(gains.kD);
        controller.setFF(gains.kF);

        return controller;
    }

    public static final CANSparkMax lowerFollowerStatusPeriod(CANSparkMax motor) {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 300);

        return motor;
    }

    public static final CANSparkMax lowerLeaderStatusPeriod(CANSparkMax motor) {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 300);

        return motor;
    }

    public static final CANSparkMax setDefaultStatusPeriod(CANSparkMax motor) {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

        return motor;
    }

    public static SparkMaxPIDController setOutputRange(SparkMaxPIDController controller, int slotIdx, Range range) {
        controller.setOutputRange(range.min(), range.max(), slotIdx);
        return controller;
    }

    public static PIDController setGains(PIDController controller, Gains gains) {
        controller.setPID(gains.kP, gains.kI, gains.kD);
        return controller;
    }
}
