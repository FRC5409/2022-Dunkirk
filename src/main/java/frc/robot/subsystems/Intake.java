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
    public static enum kIntakeStates{
        kExtended, kRetracted;
    }


    private WPI_TalonFX mot_intake;
    private CANSparkMax mot_intakeIn;
    private DoubleSolenoid intakeSolenoid_left;
    private DoubleSolenoid intakeSolenoid_right;

    private kIntakeStates state;
    private double target;
    private double targetIn;

    public Intake() {
        mot_intakeIn = new CANSparkMax(kIntake.kIntakeMotorIn, MotorType.kBrushless);
        mot_intakeIn.setSmartCurrentLimit(20);
        mot_intakeIn.setIdleMode(IdleMode.kBrake);
        mot_intakeIn.setInverted(true);
        MotorUtils.lowerLeaderStatusPeriod(mot_intakeIn);

        mot_intakeIn.burnFlash();

        //intakeSolenoid_left = new DoubleSolenoid(PneumaticsModuleType.REVPH, kIntake.kLeftIntakeSolenoid1, kIntake.kLeftIntakeSolenoid2);
        //intakeSolenoid_right = new DoubleSolenoid(PneumaticsModuleType.REVPH, kIntake.kRightIntakeSolenoid1, kIntake.kRightIntakeSolenoid2);

        mot_intake = new WPI_TalonFX(kIntake.kIntakeMotor);
        mot_intake.setInverted(true);
        MotorUtils.lowerNonImportantPeriods(mot_intake);
        // mot_intake.setSmartCurrentLimit(20);
        // mot_intake.setIdleMode(IdleMode.kBrake);
        // mot_intake.burnFlash();

        //intakeSolenoid_left = new DoubleSolenoid(kID.PneumaticHub, PneumaticsModuleType.REVPH, kIntake.kLeftIntakeSolenoid1, kIntake.kLeftIntakeSolenoid2);
        intakeSolenoid_right = new DoubleSolenoid(kID.PneumaticHub, PneumaticsModuleType.REVPH, kIntake.kRightIntakeSolenoid1, kIntake.kRightIntakeSolenoid2);

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
        mot_intakeIn.set(speed);
        targetIn = speed;
    }

    public void reverseIntakeIn(double speed){
        speed *= -1;
        if(speed == targetIn) return;
        mot_intakeIn.set(speed);
        targetIn = speed;
    }


    public void solenoidsDown() {
        //intakeSolenoid_left.set(DoubleSolenoid.Value.kForward);
        if(state == kIntakeStates.kExtended) return;
        intakeSolenoid_right.set(DoubleSolenoid.Value.kForward);
        state = kIntakeStates.kExtended;
        //System.out.println("Intake down");
    }

    public void solenoidsUp() {
        //intakeSolenoid_left.set(DoubleSolenoid.Value.kReverse);
        if(state == kIntakeStates.kRetracted) return;
        intakeSolenoid_right.set(DoubleSolenoid.Value.kReverse);
        state = kIntakeStates.kRetracted;
        //System.out.println("Intake up");

    }

    public boolean isExtended() {
        return (intakeSolenoid_right.get() == DoubleSolenoid.Value.kForward);
    }

    /**
     * Method for getting the current state of the intake solenoids. 
     */
    public kIntakeStates getIntakeState(){
        return state;
    }
    
}
