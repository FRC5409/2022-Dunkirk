// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.kIntake;
import frc.robot.Constants.kIndexer;

import java.util.HashMap;
import java.util.Map;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeIndexer extends SubsystemBase {
  private CANSparkMax mot_intake;
  private DoubleSolenoid intakeSolenoid_left;
  private DoubleSolenoid intakeSolenoid_right;

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

  protected TimeOfFlight TOF_Ent;
  protected TimeOfFlight TOF_Ext;

  protected boolean isRangeValid_Ent;
  protected boolean isRangeValid_Ext;

  protected double getRange_Ent;
  protected double getRange_Ext;

  protected final static int COMMAND_REGISTER_BIT = 0x80;

  //indexer testing motors
  protected final CANSparkMax indexerBelt_neo; 
  protected final CANSparkMax indexerShooter_neo; 

  //shuffleboard values
  HashMap<String, NetworkTableEntry> shuffleBoardFields;
  ShuffleboardTab tab;

  double speedBelt = 0; 
  double speedShoot = 0; 

  public IntakeIndexer() {
    mot_intake = new CANSparkMax(kIntake.kIntakeMotor, MotorType.kBrushless);
    mot_intake.setSmartCurrentLimit(20);
    mot_intake.setIdleMode(IdleMode.kBrake);
    mot_intake.burnFlash();

    //test motor for belt on indexer prototype
    indexerBelt_neo = new CANSparkMax(kIndexer.kIndexBeltMotor, MotorType.kBrushless);
    indexerBelt_neo.setSmartCurrentLimit(20);
    indexerBelt_neo.setIdleMode(IdleMode.kBrake);
    indexerBelt_neo.burnFlash();

    //test motor for indexer to shooter (flywheel thing)
    indexerShooter_neo = new CANSparkMax(kIndexer.kIndexShooterMotor, MotorType.kBrushless); 
    indexerShooter_neo.setSmartCurrentLimit(20);
    indexerShooter_neo.setIdleMode(IdleMode.kBrake);
    indexerShooter_neo.burnFlash();

    intakeSolenoid_left = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kIntake.kLeftIntakeSolenoid1, kIntake.kLeftIntakeSolenoid2);
    intakeSolenoid_right = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kIntake.kRightIntakeSolenoid1, kIntake.kRightIntakeSolenoid2);

    mot_indexer = new CANSparkMax(kIndexer.kIndexerMotor, MotorType.kBrushless);
    mot_indexer.setSmartCurrentLimit(kIndexer.currentLimit);
    mot_indexer.setIdleMode(IdleMode.kBrake);
    mot_indexer.burnFlash();

    m_colorMatcher_etr.addColorMatch(kBlueTarget);
    m_colorMatcher_etr.addColorMatch(kRedTarget);

    m_colorMatcher_ext.addColorMatch(kBlueTarget);
    m_colorMatcher_ext.addColorMatch(kRedTarget);


    //shuffleboard values
    shuffleBoardFields = new HashMap<String, NetworkTableEntry>();
    tab = Shuffleboard.getTab("Motors");
    ShuffleboardLayout mLayout = tab.getLayout("motor layout", BuiltInLayouts.kList);
    shuffleBoardFields.put("motor speed belt",
        mLayout.add("motor speed belt", speedBelt).withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 100, "block increment", 10)).getEntry());
    
    shuffleBoardFields.put("current speed of belt", mLayout.add("Current belt speed", getSpeedBelt()).getEntry());

    shuffleBoardFields.put("motor speed shooter",
        mLayout.add("motor speed shooter", speedShoot).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 100, "block increment", 10)).getEntry());

    shuffleBoardFields.put("current speed of shoot", mLayout.add("Current shooter speed", getSpeedBelt()).getEntry());
  }

  

  //test stuff
  public void indexBeltOn(){
    indexerBelt_neo.set(speedBelt);
  }

  public void indexShootOn(){
    indexerShooter_neo.set(speedShoot);
  }

  public void setSpeedBelt(double speed){
    speedBelt = speed;
  }

  public void setSpeedShoot(double speed){
    speedShoot = speed; 
  }

  public double getSpeedBelt(){
    return speedBelt;
  }

  public double getSpeedShoot(){
    return speedShoot;
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

  public void indexerOn(double speed) {
    mot_indexer.set(speed);
  }

  public void reverseIndexer(double speed) {
    mot_indexer.set(-speed);
  }

  public char getFMS() {
    String gameData;
    gameData = DriverStation.getAlliance().name(); // name?
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          allianceColour = 'B';
          // Blue case code
          break;
        case 'R':
          allianceColour = 'R';
          // Red case code
          break;
        default:
          // This is corrupt data
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
    } else {
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
    } else {
      detectedExitColour = 'U';
    }
  }

  public char getEntranceColour() {
    return detectedEntranceColour;
  }

  public char getExitColour() {
    return detectedExitColour;
  }

  public double getRange_Ent() {
    return TOF_Ent.getRange();
  }

  public double getRange_Ext() {
    return TOF_Ext.getRange();
  }

  public boolean ballDetectionEnter() {
    double range = TOF_Ent.getRange();
    if (range < 24) { //need to find the range to compare with
      return true;
    }
    return false;
  }

  public boolean ballDetectionExit() {
    double range = TOF_Ext.getRange();

    if (range < 24) { //need to find number to compare with
      return true;
    }
    return false;
  }

  public boolean isRangeValid_Ent() {
    return TOF_Ent.isRangeValid();
  }

  public boolean isRangeValid_Ext() {
    return TOF_Ext.isRangeValid();
  }

  public void setRangingMode(TimeOfFlight.RangingMode rangeModeIn, double sampleTime) {
    if (sampleTime > 24) {
      sampleTime = 24;
      TOF_Ent.setRangingMode(rangeModeIn, sampleTime);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("TOF Enter", TOF_Ent.getRange());
    // SmartDashboard.putNumber("TOF Exit", TOF_Ext.getRange());
    // SmartDashboard.putBoolean("TOF Enter In Range", ballDetectionEnter());
    // SmartDashboard.putBoolean("TOF Exit In Range", ballDetectionExit());
    //SmartDashboard.putData(getFMS());

    setSpeedBelt(shuffleBoardFields.get("motor speed belt").getDouble(50));
    shuffleBoardFields.get("current speed of belt").setDouble(getSpeedBelt());

    setSpeedShoot(shuffleBoardFields.get("motor speed shoot").getDouble(50));
    shuffleBoardFields.get("current speed shoot").setDouble(getSpeedShoot());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
