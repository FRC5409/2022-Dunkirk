package frc.robot.subsystems.shooter;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
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
import frc.robot.utils.Toggleable;

/**
 * 
 * @author Akil Pathiranage, Keith Davies
 */
public class ShooterTurret extends SubsystemBase implements Toggleable {
    public static enum ReferenceType {
        kRotation(0), kTracking(1);

        public int idx;

        private ReferenceType(int idx) {
            this.idx = idx;
        }
    }

    private static final IdleMode DEFAULT_IDLE_MODE = IdleMode.kBrake;
    // private static final Range ROTATION_RANGE = Constants.Shooter.ROTATION_RANGE;
    // private static final Range MANUAL_ROTATION_RANGE = Constants.Shooter.MANUAL_ROTATION_RANGE;

    // private static final double ROTATIONS_PER_DEGREE = (Constants.Shooter.GEAR_RATIO / 360); // Rotations per degree
    // private static final double DEGREES_PER_ROTATION = 1 / ROTATIONS_PER_DEGREE; // Degrees per rotation

    private final HashMap<String, NetworkTableEntry> fields;

    // private WPI_TalonFX mot_main;
    private final CANSparkMax                        mot_main;
    private final RelativeEncoder                    enc_main;
    private final SparkMaxPIDController              ctr_main;
    
    private final DoubleSolenoid                     dsl_hood; 
    private final DigitalInput                       lim_zero;

    private ReferenceType                            referenceType;
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
            mot_main.setIdleMode(DEFAULT_IDLE_MODE);
            mot_main.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Shooter.ROTATION_RANGE.min());
            mot_main.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Shooter.ROTATION_RANGE.max());

        enc_main = mot_main.getEncoder();
            enc_main.setPosition(0);
            enc_main.setPositionConversionFactor(360.0 / Constants.Shooter.GEAR_RATIO);

        ctr_main = mot_main.getPIDController();
        MotorUtils.setGains(ctr_main, ReferenceType.kRotation.idx, Constants.Shooter.TURRET_ROTATION_GAINS);
        MotorUtils.setOutputRange(ctr_main, ReferenceType.kRotation.idx, Constants.Shooter.TURRET_OUTPUT_RANGE);

        MotorUtils.setGains(ctr_main, ReferenceType.kTracking.idx, Constants.Shooter.TURRET_TRACKING_GAINS);
        MotorUtils.setOutputRange(ctr_main, ReferenceType.kTracking.idx, Constants.Shooter.TURRET_OUTPUT_RANGE);

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
        rotation = 0;
        enabled = false;
        target = 0;
    }

    /**
     * Method for enabling the turret subsystem.
     */
    @Override
    public void enable() {
        enabled = true;
    }

    /**
     * Method for disabling the turret subsystem.
     */
    @Override
    public void disable() {
        if (!enabled) return;

        mot_main.setIdleMode(DEFAULT_IDLE_MODE);
        mot_main.disable();

        enabled = false;
    }

    
    /**
     * {@inheritDoc}
     */
    @Override
    public void periodic() {
        if (enabled) {
            rotation = enc_main.getPosition();
            
            if(lim_zero.get() == Constants.Shooter.ZERO_LIMIT_POLARITY) {
                if (!calibrated) {
                    enc_main.setPosition(0);
                    calibrated = true;
                }
            } else {
                if (calibrated)
                    calibrated = false;
            }
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
     * Method for setting the rotation target of the turret.
     * 
     * @param value Angle to turn to in degrees, can be negative for turning
     *                  left, positive for turning right.
     * 
     * @deprecated {@link #setReference(double, ReferenceType) setReference(double, ReferenceType.kRotation)}
     */
    @Deprecated
    public void setRotationTarget(double value) {
        setReference(value, ReferenceType.kRotation);
    }
    
    public void setReference(double value, ReferenceType type) {
        setReference(value, 0, type);
    }

    public void setReference(double value, double feedForwardValue, ReferenceType type) {
        if (!enabled) return;
        
        ctr_main.setReference(
            Constants.Shooter.ROTATION_RANGE.clamp(value), 
            CANSparkMax.ControlType.kPosition,
            type.idx, feedForwardValue,
            SparkMaxPIDController.ArbFFUnits.kPercentOut);

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
        return isTargetReached(Constants.Shooter.ALIGNMENT_THRESHOLD);
    }
    
    /**
     * Method for getting if the turret is aligned with its target.
     * 
     * @return True if its aligned, false if not.
     */
    public boolean isTargetReached(double threshold) {
        return enabled ? Math.abs(rotation - target) < threshold : false;
    }
    
    
    /**
     * Method for getting if the turret has been enabled. 
     * @return true if the turret has been enabled, false if not.
     */
    @Override
    public boolean isEnabled() {
        return enabled;
    }
}
