package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kID;
import frc.robot.Constants.kIntake;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;



public class Intake extends SubsystemBase{

    private WPI_TalonFX mot_intake;
    private CANSparkMax mot_intakeIn;
    private DoubleSolenoid intakeSolenoid_left;
    private DoubleSolenoid intakeSolenoid_right;

    public Intake() {

       

        mot_intakeIn = new CANSparkMax(kIntake.kIntakeMotorIn, MotorType.kBrushless);
        mot_intakeIn.setSmartCurrentLimit(20);
        mot_intakeIn.setIdleMode(IdleMode.kBrake);
        mot_intakeIn.setInverted(true);
        mot_intakeIn.burnFlash();

        //intakeSolenoid_left = new DoubleSolenoid(PneumaticsModuleType.REVPH, kIntake.kLeftIntakeSolenoid1, kIntake.kLeftIntakeSolenoid2);
        //intakeSolenoid_right = new DoubleSolenoid(PneumaticsModuleType.REVPH, kIntake.kRightIntakeSolenoid1, kIntake.kRightIntakeSolenoid2);

        mot_intake = new WPI_TalonFX(kIntake.kIntakeMotor);
        mot_intake.setInverted(true);
        // mot_intake.setSmartCurrentLimit(20);
        // mot_intake.setIdleMode(IdleMode.kBrake);
        // mot_intake.burnFlash();

        //intakeSolenoid_left = new DoubleSolenoid(kID.PneumaticHub, PneumaticsModuleType.REVPH, kIntake.kLeftIntakeSolenoid1, kIntake.kLeftIntakeSolenoid2);
        intakeSolenoid_right = new DoubleSolenoid(kID.PneumaticHub, PneumaticsModuleType.REVPH, kIntake.kRightIntakeSolenoid1, kIntake.kRightIntakeSolenoid2);

        solenoidsUp();
    }

    public void intakeOn(double speed) {
        mot_intake.set(speed);
    }

    public void reverseIntake(double speed) {
        mot_intake.set(-speed);
    }


    public void intakeIn(double speed){
        mot_intakeIn.set(speed);
    }

    public void reverseIntakeIn(double speed){
        mot_intakeIn.set(-speed);
    }


    public void solenoidsDown() {
        //intakeSolenoid_left.set(DoubleSolenoid.Value.kForward);
        intakeSolenoid_right.set(DoubleSolenoid.Value.kForward);
        System.out.println("Intake down");
    }

    public void solenoidsUp() {
        //intakeSolenoid_left.set(DoubleSolenoid.Value.kReverse);
        intakeSolenoid_right.set(DoubleSolenoid.Value.kReverse);
        System.out.println("Intake up");

    }

    public boolean isExtended() {
        return (intakeSolenoid_right.get() == DoubleSolenoid.Value.kForward);
    }
    
}
