package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kID;
import frc.robot.Constants.kIntake;
import frc.robot.utils.MotorUtils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;



public class Intake extends SubsystemBase{

    /**
     * Constants representing the current state of the 
     * solenoids for the intake. This is used to prevent
     * flooding of the CAN bus. 
     */
    public static enum IntakeState {
        kExtended, kRetracted;
    }


    private WPI_TalonFX mot_intake;
    private CANSparkMax mot_sequencer;
    private DoubleSolenoid sol_intake;

    private IntakeState state;

    private double target;
    private double targetIn;

    public Intake() {
        mot_sequencer = new CANSparkMax(kIntake.kIntakeMotorIn, MotorType.kBrushless);
            mot_sequencer.setSmartCurrentLimit(20);
            mot_sequencer.setIdleMode(IdleMode.kBrake);
            mot_sequencer.setInverted(true);

        MotorUtils.lowerLeaderStatusPeriod(mot_sequencer);
        mot_sequencer.burnFlash();

        mot_intake = new WPI_TalonFX(kIntake.kIntakeMotor);
        mot_intake.setInverted(true);
        MotorUtils.lowerNonImportantPeriods(mot_intake);
        
        sol_intake = new DoubleSolenoid(
            kID.PneumaticHub, PneumaticsModuleType.REVPH, kIntake.kRightIntakeSolenoid1, kIntake.kRightIntakeSolenoid2);

        solenoidsUp();
    }

    public void intakeOn(double speed) {
        if (speed == target) return;

        mot_intake.set(speed);
        target = speed;
    }

    public void reverseIntake(double speed) {
        speed *= -1;
        if (speed == target) return;
        mot_intake.set(speed);
        target = speed;
    }


    public void intakeIn(double speed){
        if (speed == targetIn) return;
        mot_sequencer.set(speed);
        targetIn = speed;
    }

    public void reverseIntakeIn(double speed){
        speed *= -1;
        if(speed == targetIn) return;
        mot_sequencer.set(speed);
        targetIn = speed;
    }


    public void solenoidsDown() {
        if (state == IntakeState.kExtended)
            return;

        sol_intake.set(DoubleSolenoid.Value.kForward);
        state = IntakeState.kExtended;
    }

    public void solenoidsUp() {
        if (state == IntakeState.kRetracted)
            return;

        sol_intake.set(DoubleSolenoid.Value.kReverse);
        state = IntakeState.kRetracted;

    }

    public boolean isExtended() {
        return (sol_intake.get() == DoubleSolenoid.Value.kForward);
    }

    /**
     * Method for getting the current state of the intake solenoids. 
     */
    public IntakeState getIntakeState(){
        return state;
    }
    
}
