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
import frc.robot.utils.MotorUtils;
import frc.robot.utils.Toggleable;

/**
 * 
 * @author Akil Pathiranage, Keith Davies
 */
public class ShooterTurret extends SubsystemBase implements Toggleable {
    public static final double ROTATIONS_PER_DEGREE = (Constants.Shooter.GEAR_RATIO / 360); // Rotations per degree
    public static final double DEGREES_PER_ROTATION = 1 / ROTATIONS_PER_DEGREE; // Degrees per rotation

    private final HashMap<String, NetworkTableEntry> fields;

    // private WPI_TalonFX mot_main;
    private final CANSparkMax                        mot_main;
    private final RelativeEncoder                    enc_main;
    private final SparkMaxPIDController              ctr_main;
    
    private final DoubleSolenoid                     dsl_hood; 
    private final DigitalInput                       lim_zero;

    private boolean                                  calibrated;
    private double                                   target;
    private boolean                                  enabled;
    

    /**
     * Constructor for the turret.
     */
    public ShooterTurret() {
        mot_main = new CANSparkMax(Constants.kID.TurretNeo, MotorType.kBrushless);
            mot_main.restoreFactoryDefaults();
            mot_main.setIdleMode(IdleMode.kCoast);

        enc_main = mot_main.getEncoder();
            enc_main.setPosition(0);

        ctr_main = mot_main.getPIDController();
            ctr_main.setOutputRange(-0.5, 0.5);
        MotorUtils.setGains(ctr_main, Constants.Shooter.TURRET_GAINS);

        lim_zero = new DigitalInput(Constants.kID.TurretLimitSwitch1);

        dsl_hood = new DoubleSolenoid(
            Constants.kID.PneumaticHub, PneumaticsModuleType.REVPH, 
            Constants.Shooter.HOOD_FORWARD_CHANNEL, Constants.Shooter.HOOD_REVERSE_CHANNEL);

        fields = new HashMap<String, NetworkTableEntry>();
        fields.put("hood", Shuffleboard.getTab("Turret")
            .add("Hood position", "off").getEntry());
        fields.put("rotation", Shuffleboard.getTab("Turret")
            .add("Turret Rotation", 0).getEntry());

        enabled = false;
        target = 0;
        calibrated = false;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void periodic() {
        if(lim_zero.get() == Constants.Shooter.ZERO_LIMIT_POLARITY) {
            if (!calibrated) {
                enc_main.setPosition(0);
                calibrated = true;
            }
        } else {
            if (!calibrated)
                calibrated = false;
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

        fields.get("rotation").setNumber(getRotation());
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
     * Method for disabling the turret subsystem.
     */
    public void disable() {
        enabled = false;
        mot_main.disable();
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
     */
    public void setRotationTarget(double value) {
        if (!enabled) return;
        
        // TODO add safety
        ctr_main.setReference(
            ROTATIONS_PER_DEGREE * Constants.Shooter.ROTATION_RANGE.clamp(value), 
            CANSparkMax.ControlType.kPosition);
        
        target = value;
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
        return enc_main.getPosition() * DEGREES_PER_ROTATION;
    }

    /**
     * Method for getting if the turret is aligned with its target.
     * 
     * @return True if its aligned, false if not.
     */
    public boolean isTargetReached() {
        return Math.abs(getRotation() - target) < Constants.Vision.ALIGNMENT_THRESHOLD;
    }


    /**
     * Method for setting the hood to the up position.
     */
    public void hoodUpPosition() {
        if(!enabled) return;
        dsl_hood.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Method for setting the hood to the down position.
     */
    public void hoodDownPosition() {
        if(!enabled) return;
        dsl_hood.set(DoubleSolenoid.Value.kReverse);
    }

}
