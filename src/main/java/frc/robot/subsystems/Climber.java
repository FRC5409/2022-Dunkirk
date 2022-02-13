// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.MoveClimberArm;

public class Climber extends SubsystemBase {
  private CANSparkMax mot_main;
  private CANSparkMax mot_follower;
  private SparkMaxPIDController controller_main;
  private RelativeEncoder encoder_main;

  private boolean locked;
  private HashMap<String, NetworkTableEntry> shuffleboardFields;

  private final DigitalInput limitSwitch;


  /**
   * Constructor for the climber.
   */
  public Climber() {
    mot_main = new CANSparkMax(Constants.kClimber.CAN_MASTER_MOT, MotorType.kBrushless);
    mot_main.setInverted(false);
    mot_main.setIdleMode(IdleMode.kCoast);

    mot_follower = new CANSparkMax(Constants.kClimber.CAN_FOLLOWER_MOT, MotorType.kBrushless);
    mot_follower.follow(mot_main, true);
    mot_follower.setIdleMode(IdleMode.kCoast);


    // encoder
    encoder_main = mot_main.getEncoder();
    zeroEncoder();

    mot_main.burnFlash();
    mot_follower.burnFlash();

    controller_main = mot_main.getPIDController();
    controller_main.setOutputRange(-1, 1);
    configPID(Constants.kClimber.P, Constants.kClimber.I, Constants.kClimber.D, Constants.kClimber.F);

    locked = false;

    limitSwitch = new DigitalInput(Constants.kClimber.DIGITAL_INPUT_PORT);;

    // Gives absolute motor positions of 0 - 360 degrees, all positive values.
    // mot_main.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
    shuffleboardFields = new HashMap<String, NetworkTableEntry>();

    ShuffleboardLayout layout = Shuffleboard.getTab("Climber").getLayout("Movement", BuiltInLayouts.kList);
    layout.add("ELEVATE", new MoveClimberArm(this)).withWidget(BuiltInWidgets.kCommand);

    shuffleboardFields.put("toPosition", layout.add("TO POSITION", 0).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 104)).getEntry());

    shuffleboardFields.put("currentPos",
        layout.add("CURRENT POSITION", 0).withWidget(BuiltInWidgets.kTextView).getEntry());
  }

  private void zeroEncoder() {
    encoder_main.setPosition(0);
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void periodic() {
    shuffleboardFields.get("currentPos").setNumber(getPosition());
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void simulationPeriodic() {
  }

  /**
   * Method for retracting or extending the climber arm.
   */
  public void moveArm(double target) {

    // TODO currently takes in a fixed rate.
    if (!locked) {
      // currently moves the motor at a rate of 180 degrees per 100ms.
      controller_main.setReference(target, ControlType.kPosition);
    }
  }

  /**
   * Method for locking the arm.
   */
  public void lockArm() {

    locked = true;
  }

  /**
   * Method for unlocking the arm.
   */
  public void unlockArm() {
    locked = false;
  }

  /**
   * This method can be called to toglle the locked value.
   */
  public void toggleLock() {
    locked = !locked;
  }

  /**
   * Method for getting the position of the arm extended. This is a calculated
   * value.
   * 
   * @return The position at which the arm is currently extended.
   */
  public double getPosition() {
    return encoder_main.getPosition();
  }

  public void configPID(double P, double I, double D, double F) {
    controller_main.setP(P);
    controller_main.setI(I);
    controller_main.setD(D);
    controller_main.setFF(F);
  }

  public double getSliderPosition() {
    return shuffleboardFields.get("toPosition").getDouble(0);
  }

  public void disableMotors() {
    mot_main.disable();
    mot_follower.disable();
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  public void findZero() {
    controller_main.setReference(-10.0, ControlType.kVelocity);
  }
}
