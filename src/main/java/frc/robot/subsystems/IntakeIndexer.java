// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.kIntake;
import frc.robot.Constants.kIndexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeIndexer extends SubsystemBase {
  private CANSparkMax mot_intake;
  //private DoubleSolenoid intakeSolenoid_left;
  //private DoubleSolenoid intakeSolenoid_right;

  private CANSparkMax mot_indexer;

  private I2C.Port i2cPort_1 = I2C.Port.kOnboard;
  private ColorSensorV3 m_colourSensor_etr = new ColorSensorV3(i2cPort_1);
  private ColorMatch m_colorMatcher_etr = new ColorMatch();

  private I2C.Port i2cPort_2 = I2C.Port.kOnboard;
  private ColorSensorV3 m_colourSensor_ext = new ColorSensorV3(i2cPort_2);
  private ColorMatch m_colorMatcher_ext = new ColorMatch();

  // need to be retested for the value
  private Color kBlueTarget = new Color(0.120, 0.402, 0.479);
  private Color kRedTarget = new Color(0.532, 0.330, 0.137);

  private char allianceColour;
  private char detectedEntranceColour;
  private char detectedExitColour;


  public IntakeIndexer() {
    mot_intake = new CANSparkMax(kIntake.kIntakeMotor, MotorType.kBrushless);	
    mot_intake.setSmartCurrentLimit(20); 	
    mot_intake.setIdleMode(IdleMode.kBrake);	
    mot_intake.burnFlash();  	

    intakeSolenoid_left = new DoubleSolenoid(null, kIntake.kLeftIntakeSolenoid1, kIntake.kLeftIntakeSolenoid2);
    intakeSolenoid_right = new DoubleSolenoid(null, kIntake.kRightIntakeSolenoid1, kIntake.kRightIntakeSolenoid2);	

    mot_indexer = new CANSparkMax(kIndexer.kIndexerMotor, MotorType.kBrushless);
		mot_indexer.setSmartCurrentLimit(kIndexer.currentLimit);
		mot_indexer.setIdleMode(IdleMode.kBrake);
		mot_indexer.burnFlash();

    m_colorMatcher_etr.addColorMatch(kBlueTarget);
    m_colorMatcher_etr.addColorMatch(kRedTarget);

    m_colorMatcher_ext.addColorMatch(kBlueTarget);
    m_colorMatcher_ext.addColorMatch(kRedTarget);

  }

  public void intakeOn(double speed){
    mot_intake.set(speed);
  }

  public void reverseIntake(double speed){
    mot_intake.set(-speed);
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

	public void indexerOn(double speed){
		mot_indexer.set(speed);
	}

  public void reverseIndexer(double speed){
    mot_indexer.set(-speed);
  }

  public char getFMS() {
    String gameData;
    gameData = DriverStation.getAlliance().name(); // name?
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
        allianceColour = 'B';
          //Blue case code
          break;
        case 'R' :
        allianceColour = 'R';
          //Red case code
          break;
        default :
          //This is corrupt data
          break;
      }
    }
    return allianceColour;
  }

  // to get colour value (entrance colour sensor)
  public void entranceColourTest() {

    final Color detectedColour = m_colourSensor_etr.getColor();
    ColorMatchResult match = m_colorMatcher_etr.matchClosestColor(detectedColour);

    final double IR = m_colourSensor_etr.getIR();
    final int proximity = m_colourSensor_etr.getProximity();

    SmartDashboard.putNumber("Entrance blue value", match.color.blue);
    SmartDashboard.putNumber("Entrance red value", match.color.red);
    SmartDashboard.putNumber("Entrance green value", match.color.green);
    SmartDashboard.putNumber("Entrance confidence", match.confidence);
    SmartDashboard.putNumber("Entrance proximity", proximity);
    SmartDashboard.putNumber("Entrance IR", IR);

  }

  // to get colour value (entrance colour sensor)
  public void exitColourTest() {

    final Color detectedColour = m_colourSensor_ext.getColor();
    ColorMatchResult match = m_colorMatcher_ext.matchClosestColor(detectedColour);

    final double IR = m_colourSensor_ext.getIR();
    final int proximity = m_colourSensor_ext.getProximity();

    SmartDashboard.putNumber("Exit blue value", match.color.blue);
    SmartDashboard.putNumber("Exit red value", match.color.red);
    SmartDashboard.putNumber("Exit green value", match.color.green);
    SmartDashboard.putNumber("Exit confidence", match.confidence);
    SmartDashboard.putNumber("Exit proximity", proximity);
    SmartDashboard.putNumber("Exit IR", IR);

  }

  public void entranceColourCalibration() {

    final Color detectedColor = m_colourSensor_etr.getColor();
    ColorMatchResult match = m_colorMatcher_etr.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      detectedEntranceColour = 'B';
    } else if (match.color == kRedTarget) {
      detectedEntranceColour = 'R';
    } else{
      detectedEntranceColour = 'U';
    }
  }

  public void exitColourCalibration() {

    final Color detectedColor = m_colourSensor_ext.getColor();
    ColorMatchResult match = m_colorMatcher_ext.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      detectedExitColour = 'B';
    } else if (match.color == kRedTarget) {
      detectedExitColour = 'R';
    } else{
      detectedExitColour = 'U';
    }
  }

  public char getEntranceColour(){
    return detectedEntranceColour;
  }

  public char getExitColour(){
    return detectedExitColour;
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
