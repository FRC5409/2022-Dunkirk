package frc.robot.subsystems;

import frc.robot.Constants.kIndexer;
import frc.robot.utils.Toggleable;

import com.playingwithfusion.TimeOfFlight;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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


  protected final static int COMMAND_REGISTER_BIT = 0x80;


  double speedBelt = 0;
  double speedShoot = 0;

  private boolean enabled;
  
  int countBalls = 0; 
  boolean shooterReady = false; 

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
    indexerBelt_neo = new CANSparkMax(kIndexer.indexerMotorBottom, MotorType.kBrushless);
    indexerBelt_neo.setSmartCurrentLimit(kIndexer.currentLimit);
    indexerBelt_neo.setIdleMode(IdleMode.kBrake);
    indexerBelt_neo.setInverted(true);
    indexerBelt_neo.burnFlash();
  }

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
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TOF Enter", TOF_Ent.getRange());
    SmartDashboard.putNumber("TOF Exit", TOF_Ext.getRange());
    SmartDashboard.putNumber("TOF Ball1", TOF_Ball1.getRange());


    if(ballDetectionEnter() == true){
      countBalls = 1; 
    } else if(ballDetectionBall1() == true && !ballDetectionEnter()){
      countBalls = 2; 
    } else if (!ballDetectionExit() && !ballDetectionEnter() && !ballDetectionBall1()){
      countBalls = 0; 
    }

    if(ballDetectionExit() == true){
      shooterReady = true; 
    } else {
      shooterReady = false; 
    }

    SmartDashboard.putNumber("Number of Balls in Indexer", countBalls); 
    SmartDashboard.putBoolean("Ready to Shoot", shooterReady);
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

  public double getRange_Ext() {
    return TOF_Ext.getRange();
  }

  public double getRange_Ent() {
    return TOF_Ent.getRange();
  }

  public double getRange_Ball1(){
    return TOF_Ball1.getRange();
  }
 
  public double getSpeedBelt() {
    return speedBelt;
  }

  public void indexerOn(double speed) {
    indexerBelt_neo.set(speed);
  }

  public void reverseIndexer(double speed) {
    indexerBelt_neo.set(-speed);
  }

  public boolean ballDetectionEnter() {
    double range = TOF_Ent.getRange();
    if (range < kIndexer.rangeEnter_1) { // need to find the range to compare with
      return true;
    }
    return false;
  }

  public boolean ballDetectionBall1() {
    double range = TOF_Ball1.getRange();
    if (range < kIndexer.rangeBall1_1) {
      return true;
    }
    return false;
  }

  public boolean ballDetectionExit() {
    double range = TOF_Ext.getRange();

    if (range < kIndexer.rangeBall1_1){ // need to find number to compare with
      return true;
    }
    return false;
  }

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

  public void setRangingMode(TimeOfFlight.RangingMode rangeModeIn, double sampleTime) {
    if (sampleTime > 24) {
      sampleTime = 24;
      TOF_Ent.setRangingMode(rangeModeIn, sampleTime);
    }
  }

  /**
   * Stops the indexer by calling stopMotor()
   */
  public void stopIndexer() {
    indexerBelt_neo.stopMotor();
  }
  
}
