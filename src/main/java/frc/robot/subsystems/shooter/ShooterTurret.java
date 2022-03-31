package frc.robot.subsystems.shooter;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTableEntry;

import frc.robot.Constants;
import frc.robot.base.shooter.HoodPosition;
import frc.robot.utils.MotorUtils;
import frc.robot.utils.Range;
import frc.robot.utils.Toggleable;

/**
 * 
 * @author Akil Pathiranage, Keith Davies
 */
public class ShooterTurret extends SubsystemBase implements Toggleable {
    public static enum ReferenceType {
        kRotation, kOutput
    }

    private static enum LimitType {
        kLeft, kRight, kNone
    }

    private static final IdleMode DEFAULT_IDLE_MODE = IdleMode.kBrake;
    private static final Range ROTATION_RANGE = Constants.Shooter.ROTATION_RANGE;
    private static final Range MANUAL_ROTATION_RANGE = Constants.Shooter.MANUAL_ROTATION_RANGE;

    private static final double ROTATIONS_PER_DEGREE = (Constants.Shooter.GEAR_RATIO / 360); // Rotations per degree
    private static final double DEGREES_PER_ROTATION = 1 / ROTATIONS_PER_DEGREE; // Degrees per rotation

    private final HashMap<String, NetworkTableEntry> fields;

    // private WPI_TalonFX mot_main;
    private final CANSparkMax                        mot_main;
    private final RelativeEncoder                    enc_main;
    private final SparkMaxPIDController              ctr_main;
    
    private final DoubleSolenoid                     dsl_hood; 
    private final DigitalInput                       lim_zero;

    private ReferenceType                            referenceType;
    private LimitType                                limitType;
    private boolean                                  calibrated;
    private boolean                                  enabled;
    private double                                   target;
    private double                                   rotation;

    /**
     * Constructor for the turret.
     */
    public ShooterTurret() {
        mot_main = new CANSparkMax(Constants.kID.TurretNeo, MotorType.kBrushless);
            mot_main.restoreFactoryDefaults();
            mot_main.setIdleMode(IdleMode.kBrake);

        enc_main = mot_main.getEncoder();
            enc_main.setPosition(0);

        ctr_main = mot_main.getPIDController();
        MotorUtils.setGains(ctr_main, Constants.Shooter.TURRET_GAINS);
        MotorUtils.setOutputRange(ctr_main, 0, Constants.Shooter.TURRET_OUTPUT_RANGE);

        lim_zero = new DigitalInput(Constants.kID.TurretLimitSwitch1);

        dsl_hood = new DoubleSolenoid(
            Constants.kID.PneumaticHub, PneumaticsModuleType.REVPH, 
            Constants.Shooter.HOOD_FORWARD_CHANNEL, Constants.Shooter.HOOD_REVERSE_CHANNEL);

        fields = new HashMap<String, NetworkTableEntry>();
        fields.put("hood", Shuffleboard.getTab("Turret")
            .add("Hood position", "off").getEntry());
        fields.put("rotation", Shuffleboard.getTab("Turret")
            .add("Turret Rotation", 0).getEntry());

        referenceType = ReferenceType.kRotation;
        calibrated = false;
        limitType = LimitType.kNone;
        rotation = 0;
        enabled = false;
        target = 0;
    }

    /**
     * Method for enabling the turret subsystem.
     */
    public void enable() {
        enabled = true;
    }


    /**
     * Brings the turret to its zero position. 
     */
    public void reset() {
        setRotationTarget(0);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void periodic() {
        if (enabled) {
            rotation = enc_main.getPosition() * DEGREES_PER_ROTATION;
            
            if(lim_zero.get() == Constants.Shooter.ZERO_LIMIT_POLARITY) {
                if (!calibrated) {
                    enc_main.setPosition(0);
                    calibrated = true;
                }
            } else {
                if (calibrated)
                    calibrated = false;
            }

            updateManualLimits();
        }

        switch(dsl_hood.get()) {
            case kForward:
                SmartDashboard.putBoolean("hoodIsUp", true);
                fields.get("hood").setString("Up");
                break;
            case kReverse:
                SmartDashboard.putBoolean("hoodIsUp", false);
                fields.get("hood").setString("Down");
                break;
            default:
                SmartDashboard.putBoolean("hoodIsUp", false);
                fields.get("hood").setString("Off");
                break;
        }
        
        fields.get("rotation").setNumber(rotation);
    }

    /**
     * Method for disabling the turret subsystem.
     */
    public void disable() {
        mot_main.setIdleMode(DEFAULT_IDLE_MODE);
        mot_main.disable();

        enabled = false;
    }

    /**
     * Method for getting if the turret has been enabled. 
     * @return true if the turret has been enabled, false if not.
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Method for setting the rotation target of the turret.
     * 
     * @param value Angle to turn to in degrees, can be negative for turning
     *                  left, positive for turning right.
     * 
     * @deprecated {@link #setReference(double, ReferenceType) setReference(double, ReferenceType.kRotation)}
     */
    @Deprecated
    public void setRotationTarget(double value) {
        if (!enabled) return;
        
        setReference(value, ReferenceType.kRotation);
    }

    public void setReference(double value, ReferenceType type) {
        if (!enabled) return;
        
        if (type == ReferenceType.kOutput) {
            if (!(limitType == LimitType.kLeft && value < 0 || limitType == LimitType.kRight && value > 0))
                ctr_main.setReference(
                    Constants.Shooter.TURRET_MANUAL_OUTPUT_RANGE.clamp(value), 
                    CANSparkMax.ControlType.kDutyCycle);
        } else {
            ctr_main.setReference(
                ROTATIONS_PER_DEGREE * ROTATION_RANGE.clamp(value), 
                CANSparkMax.ControlType.kPosition);
        }

        target = value;
        referenceType = type;
    }

    public void setIdleMode(IdleMode mode) {
        if (!enabled) return;

        mot_main.setIdleMode(mode);
    }

    public void setHoodPosition(HoodPosition position) {
        if (!enabled) return;
        
        switch(position) {
            case kUp:
                dsl_hood.set(DoubleSolenoid.Value.kForward);
                break;
            case kDown:
                dsl_hood.set(DoubleSolenoid.Value.kReverse);
                break;
        }
    }

    /**1
     * Method for getting the current rotation target.
     * 
     * @return Angle in degrees.
     */
    public double getRotationTarget() {
        return target;
    }
    
    /**
     * Method for getting the current rotation.
     * 
     * @return Angle in degrees.
     */
    public double getRotation() {
        return rotation;
    }

    /**
     * Method for getting if the turret is aligned with its target.
     * 
     * @return True if its aligned, false if not.
     */
    public boolean isTargetReached() {
        return Math.abs(rotation - target) < Constants.Vision.ALIGNMENT_THRESHOLD;
    }

    private void updateManualLimits() {
        if (rotation > MANUAL_ROTATION_RANGE.max()) {
            if (limitType != LimitType.kRight) {
                if (referenceType == ReferenceType.kOutput && target > 0)
                    mot_main.stopMotor();
                limitType = LimitType.kRight;
            }
        } else if (rotation < MANUAL_ROTATION_RANGE.min()) {
            if (limitType != LimitType.kLeft) {
                if (referenceType == ReferenceType.kOutput && target < 0)
                    mot_main.stopMotor();
                limitType = LimitType.kLeft;
            }
        } else {
            limitType = LimitType.kNone;
        }
    }
}
