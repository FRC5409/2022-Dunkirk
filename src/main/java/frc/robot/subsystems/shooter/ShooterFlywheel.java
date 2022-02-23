package frc.robot.subsystems.shooter;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Gains;
import frc.robot.utils.MotorUtils;
import frc.robot.utils.Toggleable;

/**
 * Controls and operators the Shooter Flywheel.
 * 
 * @author Akil Pathiranage
 */
public final class ShooterFlywheel extends SubsystemBase implements Toggleable {
    public static final double FLYWHEEL_ROTATION_RATIO = Constants.Falcon500.unitsPerRotation / 600.0;
    public static final double FLYWHEEL_FORWARD_RATIO = 1 / FLYWHEEL_ROTATION_RATIO;
    
    private final WPI_TalonFX     mot_main;
    private final WPI_TalonFX     mot_follower;

    private final CANSparkMax     mot_feeder;
    private final SparkMaxPIDController feeder_controller;
    private final RelativeEncoder enc_feeder;
    private double                shooterTarget;
    private double                feederTarget;
    private boolean               enabled;

    private ShuffleboardTab       tab;
    private HashMap<String, NetworkTableEntry> fields;

    /**
     * Constructs a flywheel subsystem.
     */
    public ShooterFlywheel() {
        mot_main = new WPI_TalonFX(Constants.kID.ShooterFalconMotor1);
            mot_main.configFactoryDefault();
            mot_main.setNeutralMode(NeutralMode.Coast);
        MotorUtils.setGains(mot_main, 0, Constants.ShooterFlywheel.UPPER_GAINS);

        mot_follower = new WPI_TalonFX(Constants.kID.ShooterFalconMotor2);
        mot_follower.follow(mot_main);
        mot_follower.setInverted(TalonFXInvertType.OpposeMaster);

        mot_feeder = new CANSparkMax(Constants.kID.ShooterNeo, MotorType.kBrushless);
        mot_feeder.setSmartCurrentLimit(20);
        mot_feeder.setIdleMode(IdleMode.kBrake);
        mot_feeder.burnFlash();

        enc_feeder = mot_feeder.getEncoder();
        feeder_controller = mot_feeder.getPIDController();
        MotorUtils.setGains(feeder_controller, Constants.ShooterFlywheel.FEEDER_GAINS);

    
        enabled = false;
        shooterTarget = 0;
        feederTarget = 0;
    
        SmartDashboard.putNumber("Feeder Velocity", enc_feeder.getVelocity());

        fields = new HashMap<String, NetworkTableEntry>();

        //shuffleboard setup
        tab = Shuffleboard.getTab("FlywheelTuning");
        ShuffleboardLayout pidTuningLayout = tab.getLayout("PID Tuning", BuiltInLayouts.kList);
        fields.put("P", pidTuningLayout.add("P", 0.0).getEntry());
        fields.put("I", pidTuningLayout.add("I", 0.0).getEntry());
        fields.put("D", pidTuningLayout.add("D", 0.0).getEntry());
        fields.put("F", pidTuningLayout.add("F", 0.0).getEntry());

        fields.put("target", tab.add("Target", 0).getEntry());
        fields.put("recorded", tab.add("Active Velocity", 0).getEntry());
    }

    public void configPIDS(){
        disable();
        MotorUtils.setGains(mot_main, 0, new Gains(
            fields.get("P").getDouble(0), 
            fields.get("I").getDouble(0), 
            fields.get("D").getDouble(0), 
            fields.get("F").getDouble(0), 
            0, 
            0));
    }

    public void testVelocity(){
        enable();
        setVelocity(fields.get("target").getDouble(1000));
    }


    @Override
    public void periodic() {
        fields.get("recorded").setDouble(getVelocity());
        
    }
    
    /**
     * Method for enabing the flywheel.
     */
    public void enable() {
        enabled = true;
    }

    /**
     * Method for disabling the flywheel.
     */
    public void disable() {
        enabled = false;

        mot_main.stopMotor();
        mot_feeder.stopMotor();
    }

    /**
     * Method that sets the target velocity for the flywheel.
     * 
     * @param target Target revolutions per minute.
     */
    public void setVelocity(double target) {
        if (!enabled) return;

        // 600 since its rotation speed is in position change/100ms
        shooterTarget = target;
        mot_main.set(ControlMode.Velocity, shooterTarget * FLYWHEEL_ROTATION_RATIO);
    }

    /**
     * Spins the shooter at a setpoint speed from [-1.0,1.0]
     * @param setpoint
     */
    public void set(double setpoint){
        if(!enabled) return;
        mot_main.set(ControlMode.PercentOutput, setpoint);
    }

    /**
     * Method for spinning the feeder to an RPM.
     * 
     * @param target Target RPM.
     */
    public void spinFeeder(double target) {

        if(!enabled) return;

        feederTarget = target;
        feeder_controller.setReference(feederTarget, CANSparkMax.ControlType.kVelocity);

    }

    /**
     * Spins the feeder at a setpoint speed from [-1.0,1.0]
     * @param setpoint
     */
    public void spinFeederSetpoint(double setpoint){
        if(!enabled) return;


        feeder_controller.setReference(-1*setpoint, CANSparkMax.ControlType.kDutyCycle);
    }

    /**
     * Method for getting the RPM of the main motor.
     * 
     * @return A double representing the RPM of the motor.
     */
    public double getVelocity() {
        return mot_main.getSelectedSensorVelocity() * FLYWHEEL_FORWARD_RATIO;
    }



    public boolean isTargetReached() {
        return Math.abs(shooterTarget - getVelocity()) <= Constants.ShooterFlywheel.SHOOTER_TOLERANCE;
    }

    /**
     * Method for getting if the subsystem is enabled.
     * 
     * @return True if the subsystem is enabled, false if not.
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Method for stopping the feeder wheel by calling stop motor.
     */
    public void stopFeeder() {
        mot_feeder.stopMotor();
    }

    public boolean feederReachedTarget() {
        return Math.abs(feederTarget - getFeederRpm()) <= Constants.ShooterFlywheel.FEEDER_TOLERANCE;
    }

    private double getFeederRpm() {
        return enc_feeder.getVelocity();
    }

}