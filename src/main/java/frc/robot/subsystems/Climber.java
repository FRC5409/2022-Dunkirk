// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private TalonFX mot_armDriver;
  private boolean locked;


  /**
   * Constructor for the climber.
   */
  public Climber() {

    mot_armDriver = new TalonFX(Constants.kClimber.k_mot_port);
    locked = false;

    //Gives absolute motor positions of 0 - 360 degrees, all positive values. 
    mot_armDriver.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
    
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void periodic() {
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void simulationPeriodic() {
  }

  /**
   * Method for extending the climber arm. 
   * 
   * @param rate Rate at which to extend it.
   */
  public void extendArm(double rate){

  }

  /**
   * Method for retracting the climber arm.
   * 
   * @param rate Rate at which to retract it.
   */
  public void retractArm(double rate){

  }

  /**
   * Method for locking the arm.
   */
  public void lockArm(){

    locked = true;
  }

  /**
   * Method for unlocking the arm.
   */
  public void unlockArm(){
    locked = false;
  }

  /**
   * Method for getting the length of the arm extended. This is a calculated value. 
   * @return The length at which the arm is currently extended.
   */
  public double getLength(){
    return 0;
  }


}
