package frc.robot.subsystems;

import java.util.HashMap;
import frc.robot.Constants.kIndexer;
import frc.robot.utils.Toggleable;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;


import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase implements Toggleable{

  // indexer testing motors
  protected final CANSparkMax indexerBelt_neo;

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


  // shuffleboard values
  HashMap<String, NetworkTableEntry> shuffleBoardFields;
  ShuffleboardTab tab;

  double speedBelt = 0;
  double speedShoot = 0;

  private boolean enabled;

  public Indexer() {

    enabled = false;

    TOF_Ent = new TimeOfFlight(kIndexer.TOF_Ent);
    TOF_Ball1 = new TimeOfFlight(kIndexer.TOF_Ball1);
    TOF_Ext = new TimeOfFlight(kIndexer.TOF_Ext);

    TOF_Ent.setRangingMode(TimeOfFlight.RangingMode.Short, kIndexer.sampleTime);
    TOF_Ball1.setRangingMode(TimeOfFlight.RangingMode.Short, kIndexer.sampleTime);
    TOF_Ext.setRangingMode(TimeOfFlight.RangingMode.Short, kIndexer.sampleTime);

    // MOTORS
    // --------------------------------------------------------------------------------------------


    // test motor for belt on indexer prototype
    indexerBelt_neo = new CANSparkMax(kIndexer.kIndexBeltMotor, MotorType.kBrushless);
    indexerBelt_neo.setSmartCurrentLimit(20);
    indexerBelt_neo.setIdleMode(IdleMode.kBrake);
    indexerBelt_neo.burnFlash();



    TOF_Ext = new TimeOfFlight(0);
  }

  // INDEXER METHODS
  // ------------------------------------------------------------------------------------

  public void moveIndexerBelt(double speed) {
    indexerBelt_neo.set(speed);
  }

  /**
   * set the speed of the indexer belt
   */

  public void indexBeltOn() {
    indexerBelt_neo.set(speedBelt);
  }
  

  /**
   * @param speed speed of belt motor
   *              sets speed of the belt motor
   */
  public void setSpeedBelt(double speed) {
    speedBelt = speed;
  }


  @Override
  public void periodic() {

  }


  /**
   * is the pre-shooter enabled
   * 
   * @return preshooterEnabled
   */

  public boolean isEnabled() {
    return enabled;
  }


  @Override
  public void enable() {
    enabled = true;
    
  }

  @Override
  public void disable() {
    enabled = false;
    indexerBelt_neo.stopMotor();
    
  }

  /**
   * Method for psinning the lower part of the indexer.
   * 
   * @param target Setpoint speed for the lower indexer, in range [-1.0, 1.0].
   */
  public void spinIndexer(double target) {
    if(!enabled) return;
    speedBelt = target;
    indexerBelt_neo.set(speedBelt);
  }


  // TIME OF FLIGHT METHODS
  // ----------------------------------------------------------------------------

  /**
   * returns the range of the TOF
   * 
   * @return TOF_Exit.getRange()
   */
  public double getRange_Ext() {
    return TOF_Ext.getRange();
  }

  /**
   * returns the range of the TOF
   * 
   * @return TOF_Ball1.getRange()
   */
  public double getRange_Ball1() {
    return TOF_Ball1.getRange();
  }

  /**
   * returns the range of the TOF
   * 
   * @return TOF_Ent.getRange()
   */
  public double getRange_Ent() {
    return TOF_Ent.getRange();
  }

  /**
   * detects whether or not the ball is in range
   * 
   * @return true/false
   */
  public boolean ballDetectionExit() {
    double range = TOF_Ext.getRange();

    if (range < 24) { // need to find number to compare with
      return true;
    }
    return false;
  }

  /**
   * detects whether or not the ball is in range
   * 
   * @return true/false
   */
  public boolean ballDetectionBall1() {
    double range = TOF_Ball1.getRange();

    if (range < 24) {
      return true;
    }

    return false;

  }

  /**
   * detects whether the ball is in range or not
   * 
   * @return true/false
   */
  public boolean ballDetectionEnter() {
    double range = TOF_Ent.getRange();

    if (range < 24) {

      return true;
    }
    return false;
  }

  /**
   * checks whether the range is valid
   * 
   * @return TOF_Exit.isRangeValid()
   */
  public boolean isRangeValid_Ext() {
    return TOF_Ext.isRangeValid();
  }

  /**
   * checks whether the range is valid
   * 
   * @return TOF_Ball1.isRangeValid()
   */
  public boolean isRangeValid_Ball1() {
    return TOF_Ball1.isRangeValid();
  }

  /**
   * checks whether the range is valid or not
   * 
   * @return TOF_Ent.isRangeValid()
   */
  public boolean isRangeValid_Ent() {
    return TOF_Ent.isRangeValid();
  }

  /**
   * sets ranging mode
   * 
   * @param rangeModeIn
   * @param sampleTime
   */
  public void setRangingMode(TimeOfFlight.RangingMode rangeModeIn, double sampleTime) {
    if (sampleTime > 24) {
      sampleTime = 24;
      TOF_Ext.setRangingMode(rangeModeIn, sampleTime);
    }
  }

  /**
   * Stops the indexer by calling stopMotor()
   */
  public void stopIndexer() {
    indexerBelt_neo.stopMotor();
  }
}
