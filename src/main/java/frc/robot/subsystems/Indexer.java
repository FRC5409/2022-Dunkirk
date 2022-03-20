package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utils.MotorUtils;
import frc.robot.utils.Toggleable;

import com.playingwithfusion.TimeOfFlight;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * @author Elizabeth Caporiccio, Keith Davies
 */
public class Indexer extends SubsystemBase implements Toggleable {
    public static enum SensorType {
        kEnter, kExit, kBall1
    }

    public static final int INDEXER_STATE_NONE   = 0x0;
    public static final int INDEXER_STATE_ENTER  = 0x1;
    public static final int INDEXER_STATE_EXIT   = 0x2;
    public static final int INDEXER_STATE_BALL1  = 0x4;

    
    // indexer testing motors
    private final SparkMaxPIDController pid_indexerBelt;
    private final RelativeEncoder       enc_indexerBelt;
    private final CANSparkMax           mot_indexerBelt;

    // time of flights
    private final TimeOfFlight          tof_exit;
    private final TimeOfFlight          tof_enter;
    private final TimeOfFlight          tof_ball1;


    private boolean                     enabled;
    private int                         count;
    private int                         state;

    public Indexer() {
        tof_enter = new TimeOfFlight(Constants.Indexer.TOF_ENTER_CHANNEL);
            tof_enter.setRangingMode(TimeOfFlight.RangingMode.Short, Constants.Indexer.SAMPLE_TIME);
        
        tof_exit = new TimeOfFlight(Constants.Indexer.TOF_EXIT_CHANNEL);
            tof_exit.setRangingMode(TimeOfFlight.RangingMode.Short, Constants.Indexer.SAMPLE_TIME);

        tof_ball1 = new TimeOfFlight(Constants.Indexer.TOF_BALL_1_CHANNEL);
            tof_ball1.setRangingMode(TimeOfFlight.RangingMode.Short, Constants.Indexer.SAMPLE_TIME);


        // MOTORS
        // --------------------------------------------------------------------------------------------
        
        mot_indexerBelt = new CANSparkMax(Constants.Indexer.INDEXER_MOTOR_ID, MotorType.kBrushless);
            mot_indexerBelt.setSmartCurrentLimit(Constants.Indexer.CURRENT_LIMIT);
            mot_indexerBelt.setIdleMode(IdleMode.kBrake);
            mot_indexerBelt.setInverted(true);
            
        MotorUtils.lowerLeaderStatusPeriod(mot_indexerBelt);
        mot_indexerBelt.burnFlash();

        pid_indexerBelt = mot_indexerBelt.getPIDController();
        enc_indexerBelt = mot_indexerBelt.getEncoder();

        enabled = false;
        count = 0;
    }

    @Override
    public void enable() {
        enabled = true;
    }


    @Override
    public void disable() {
        if (!enabled) return;

        mot_indexerBelt.stopMotor();

        enabled = false;
    }

    @Override
    public void periodic() {
        // update state
        state = (
            (tof_enter.getRange() < Constants.Indexer.RANGE_ENTER  ? INDEXER_STATE_ENTER : 0) |
            (tof_exit.getRange() < Constants.Indexer.RANGE_EXIT    ? INDEXER_STATE_EXIT  : 0) |
            (tof_ball1.getRange() < Constants.Indexer.RANGE_BALL_1 ? INDEXER_STATE_BALL1 : 0)
        );

        if ((state & INDEXER_STATE_ENTER) != 0)
            count = 1;
        else if ((state & INDEXER_STATE_BALL1) != 0)
            count = 2;
        else
            count = 0;

        if(Constants.kConfig.DEBUG) {
            SmartDashboard.putNumber("TOF Enter", tof_enter.getRange());
            SmartDashboard.putNumber("TOF Exit", tof_exit.getRange());
            SmartDashboard.putNumber("TOF Ball1", tof_ball1.getRange());
            SmartDashboard.putNumber("Number of Balls in Indexer", count);
        }
    }

    public void setSpeed(double speed) {
        mot_indexerBelt.set(speed);
    }

    public void setControlMode(double setpoint, ControlType mode) {
        pid_indexerBelt.setReference(setpoint, mode);
    }

    /**
     * Stops the indexer.
     */
    public void stop() {
        mot_indexerBelt.stopMotor();
    }

    public int getCargoCount() {
        return count;
    }

    public double getPosition() {
        return enc_indexerBelt.getPosition();
    }

    public double getSensorRange(SensorType type) {
        return getSensor(type).getRange();
    }

    public boolean getSensorState(SensorType type) {
        switch (type) {
            case kEnter: return (state & INDEXER_STATE_ENTER) != 0;
            case kExit:  return (state & INDEXER_STATE_EXIT)  != 0;
            case kBall1: return (state & INDEXER_STATE_BALL1) != 0;
        }
        return false;
    }

    public int getSensorState() {
        return state;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isSensorValid(SensorType type) {
        return getSensor(type).isRangeValid();
    }

    
    private TimeOfFlight getSensor(SensorType type) {
        switch (type) {
            case kEnter: return tof_enter;
            case kExit:  return tof_exit;
            case kBall1: return tof_ball1;
        }
        return null;
    }

}
