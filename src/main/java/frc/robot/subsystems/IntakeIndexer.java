// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.kIntakeIndexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeIndexer extends SubsystemBase {
  private CANSparkMax mot_intake;
  private DoubleSolenoid intakeSolenoid_left;
  private DoubleSolenoid intakeSolenoid_right;

  public I2C.Port i2cPort = I2C.Port.kOnboard;
  public ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  public ColorMatch m_colorMatcher = new ColorMatch();

  public Color kBlueTarget = new Color(0.120, 0.402, 0.479);
  public Color kRedTarget = new Color(0.532, 0.330, 0.137);

  public IntakeIndexer() {
    mot_intake = new CANSparkMax(kIntakeIndexer.kIntakeMotor, MotorType.kBrushless);	
    mot_intake.setSmartCurrentLimit(20); 	
    mot_intake.setIdleMode(IdleMode.kBrake);	
    mot_intake.burnFlash();  	

    intakeSolenoid_left = new DoubleSolenoid(null, kIntakeIndexer.kLeftIntakeSolenoid1, kIntakeIndexer.kLeftIntakeSolenoid2);
    intakeSolenoid_right = new DoubleSolenoid(null, kIntakeIndexer.kRightIntakeSolenoid1, kIntakeIndexer.kRightIntakeSolenoid2);	

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);

  }

  public void intakeOn(double speed){
    mot_intake.set(speed);
  }

  public void solenoidsDown(){
    intakeSolenoid_left.set(DoubleSolenoid.Value.kForward);
    intakeSolenoid_right.set(DoubleSolenoid.Value.kForward);
  }

  public void solenoidsUp(){
    intakeSolenoid_left.set(DoubleSolenoid.Value.kReverse);
    intakeSolenoid_right.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean isExtended(){
    return (intakeSolenoid_left.get()==DoubleSolenoid.Value.kForward) && (intakeSolenoid_right.get()==DoubleSolenoid.Value.kForward);
  }

  public void reverse(double speed){
    mot_intake.set(-speed);
  }

  public String colorCalibration() {

    final Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    // final double IR = m_colorSensor.getIR();
    // final int proximity = m_colorSensor.getProximity();

    if (match.color == kBlueTarget) {
      return "BLUE";
    } else if (match.color == kRedTarget) {
      return "RED";
    }
    return "NONE";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
