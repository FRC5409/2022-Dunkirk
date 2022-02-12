package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.kIntake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


public class Intake extends SubsystemBase{

    private CANSparkMax mot_intake;
    private DoubleSolenoid intakeSolenoid_left;
    private DoubleSolenoid intakeSolenoid_right;

    public Intake() {
        mot_intake = new CANSparkMax(kIntake.kIntakeMotor, MotorType.kBrushless);
        mot_intake.setSmartCurrentLimit(20);
        mot_intake.setIdleMode(IdleMode.kBrake);
        mot_intake.burnFlash();

        intakeSolenoid_left = new DoubleSolenoid(PneumaticsModuleType.REVPH, kIntake.kLeftIntakeSolenoid1, kIntake.kLeftIntakeSolenoid2);
        intakeSolenoid_right = new DoubleSolenoid(PneumaticsModuleType.REVPH, kIntake.kRightIntakeSolenoid1, kIntake.kRightIntakeSolenoid2);
    }

    public void intakeOn(double speed) {
        mot_intake.set(speed);
    }

    public void reverseIntake(double speed) {
        mot_intake.set(-speed);
    }

    public void solenoidsDown() {
        intakeSolenoid_left.set(DoubleSolenoid.Value.kForward);
        intakeSolenoid_right.set(DoubleSolenoid.Value.kForward);
    }

    public void solenoidsUp() {
        intakeSolenoid_left.set(DoubleSolenoid.Value.kReverse);
        intakeSolenoid_right.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isExtended() {
        return (intakeSolenoid_left.get() == DoubleSolenoid.Value.kForward)
            && (intakeSolenoid_right.get() == DoubleSolenoid.Value.kForward);
    }
    
}
