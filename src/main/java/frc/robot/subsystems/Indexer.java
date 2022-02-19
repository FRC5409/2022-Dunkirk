// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.kIndexer;

import java.util.HashMap;
import java.util.Map;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
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

public class Indexer extends SubsystemBase {

  // time of flights
  protected TimeOfFlight TOF_Ext;
  protected TimeOfFlight TOF_Ent;
  protected TimeOfFlight TOF_Ball1;
  protected boolean isRangeValid_Ball1;
  protected boolean isRangeValid_Ext;
  protected boolean isRangeValid_Ent;
  protected double getRange_Ball1;
  protected double getRange_Ext;
  protected double getRange_Ent;

  protected final static int COMMAND_REGISTER_BIT = 0x80;

  // indexer testing motors
  protected final CANSparkMax indexerBelt_neo;

  // shuffleboard values
  HashMap<String, NetworkTableEntry> shuffleBoardFields;
  ShuffleboardTab tab;

  double speedBelt = 0;
  double speedShoot = 0;

  int countBalls = 0; 
  boolean shooterReady = false; 
  // private boolean indexerEnabled;
  // private boolean preshooterEnabled;

  public Indexer() {

    // test motor for belt on indexer prototype
    indexerBelt_neo = new CANSparkMax(kIndexer.indexerMotorBottom, MotorType.kBrushless);
    indexerBelt_neo.setSmartCurrentLimit(kIndexer.currentLimit);
    indexerBelt_neo.setIdleMode(IdleMode.kBrake);
    indexerBelt_neo.burnFlash();

    // shuffleboard values
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

  // test stuff
  public void indexBeltOn() {
    indexerBelt_neo.set(speedBelt);
  }

  public void setSpeedBelt(double speed) {
    speedBelt = speed;
  }

  public void setSpeedShoot(double speed) {
    speedShoot = speed;
  }

  public double getSpeedBelt() {
    return speedBelt;
  }

  public double getSpeedShoot() {
    return speedShoot;
  }

  public void indexerOn(double speed) {
    indexerBelt_neo.set(speed);
  }

  public void reverseIndexer(double speed) {
    indexerBelt_neo.set(-speed);
  }

  public double getRange_Ent() {
    return TOF_Ent.getRange();
  }

  public double getRange_Ext() {
    return TOF_Ext.getRange();
  }

  public boolean ballDetectionEnter() {
    double range = TOF_Ent.getRange();
    if (range < kIndexer.rangeEnter) { // need to find the range to compare with
      return true;
    }
    return false;
  }

  public boolean ballDetectionBall1() {
    double range = TOF_Ball1.getRange();
    if (range < kIndexer.rangeBall1) {
      return true;
    }
    return false;
  }

  public boolean ballDetectionExit() {
    double range = TOF_Ext.getRange();

    if (range < kIndexer.rangeBall1) { // need to find number to compare with
      return true;
    }
    return false;
  }

  public boolean isRangeValid_Ent() {
    return TOF_Ent.isRangeValid();
  }

  public boolean isRangeValid_Ball1() {
    return TOF_Ball1.isRangeValid();
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
    SmartDashboard.putNumber("TOF Enter", TOF_Ent.getRange());
    SmartDashboard.putNumber("TOF Exit", TOF_Ext.getRange());
    SmartDashboard.putNumber("TOF Ball1", TOF_Ball1.getRange());
    // SmartDashboard.putData(getFMS());

    setSpeedBelt(shuffleBoardFields.get("motor speed belt").getDouble(50));
    shuffleBoardFields.get("current speed of belt").setDouble(getSpeedBelt());

    setSpeedShoot(shuffleBoardFields.get("motor speed shoot").getDouble(50));
    shuffleBoardFields.get("current speed shoot").setDouble(getSpeedShoot());

    if(ballDetectionEnter() == true){
      countBalls = 1; 
    } else if(ballDetectionBall1() == true && !ballDetectionEnter()){
      countBalls = 2; 
    } 

    if(ballDetectionExit() == true){
      shooterReady = true; 
    } else {
      shooterReady = false; 
    }

    SmartDashboard.putNumber("Number of Balls in Indexer", countBalls); 
    SmartDashboard.putBoolean("Ready to Shoot", shooterReady);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
