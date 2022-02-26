package frc.robot.subsystems.shooter;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MotorUtils;
import frc.robot.utils.Toggleable;

/**
 * 
 * @author Akil Pathiranage, Keith Davies
 */
public class ShooterTurret extends SubsystemBase implements Toggleable {
    //rotations per degree
    public static final double ROTATIONS_PER_DEGREE = (Constants.Shooter.GEAR_RATIO / 360);

    //degrees per rotation
    public static final double DEGREES_PER_ROTATION = 1 / ROTATIONS_PER_DEGREE;

    // private WPI_TalonFX mot_main;
    private CANSparkMax                        mot_main;
    private RelativeEncoder                    enc_main;
    private SparkMaxPIDController              controller_main;

    private boolean                            enabled;

    private HashMap<String, NetworkTableEntry> fields;
    private double                             target;

    private DoubleSolenoid                     dsl_hood; 

    private DigitalInput                       limit_switch;
    

    /**
     * Constructor for the turret.
     */
    public ShooterTurret() {
        mot_main = new CANSparkMax(Constants.kID.TurretNeo, MotorType.kBrushless);
            mot_main.restoreFactoryDefaults();
            mot_main.setIdleMode(IdleMode.kCoast);

        enc_main = mot_main.getEncoder();
            enc_main.setPosition(0);

        controller_main = mot_main.getPIDController();
            controller_main.setOutputRange(-0.5, 0.5);

        limit_switch = new DigitalInput(Constants.kID.TurretLimitSwitch1);

        dsl_hood = new DoubleSolenoid(Constants.kID.PneumaticHub, PneumaticsModuleType.REVPH, 
        Constants.Turret.HOOD_FORWARD_CHANNEL, Constants.Turret.HOOD_REVERSE_CHANNEL);

        enabled = false;

        MotorUtils.setGains(controller_main, Constants.Shooter.TURRET_GAINS);


        fields = new HashMap<String, NetworkTableEntry>();
        fields.put("hood", Shuffleboard.getTab("Turret").add("Hood position", "off").getEntry());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void periodic() {
        if(!limit_switch.get()){
            enc_main.setPosition(0);
        }
        SmartDashboard.putNumber("encoderValue", enc_main.getPosition() * DEGREES_PER_ROTATION);
        if(dsl_hood.get().equals(Value.kForward)){
            fields.get("hood").setString("Up");
        } else if (dsl_hood.get().equals(Value.kReverse)){
            fields.get("hood").setString("Down");
        } else if (dsl_hood.get().equals(Value.kOff)){
            fields.get("hood").setString("Off");
        }

    }

    /**
     * Method for enabling the turret subsystem.
     */
    public void enable() {
        enabled = true;
    }

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
        controller_main.setReference(
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
        return Math.abs(getRotation() - target) < Constants.Turret.ALIGNMENT_TRESHOLD;
    }


    /**
     * Method for setting the hood to the up position.
     */
    public void hoodUpPosition(){
        if(!enabled) return;
        dsl_hood.set(Value.kForward);
    }

    /**
     * Method for setting the hood to the down position.
     */
    public void hoodDownPosition(){
        if(!enabled) return;
        dsl_hood.set(Value.kReverse);
    }

}
