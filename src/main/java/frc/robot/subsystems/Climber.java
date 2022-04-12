// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.Constants;
import frc.robot.Constants.kID;
import frc.robot.base.ValueProperty;
import frc.robot.commands.ElevateTo;
import frc.robot.commands.FindElevatorZero;
import frc.robot.utils.MotorUtils;

public class Climber extends SubsystemBase {
  private CANSparkMax mot_main;
  private CANSparkMax mot_follower;
  private SparkMaxPIDController controller_main;
  private RelativeEncoder encoder_main;

  private boolean locked;
  private HashMap<String, NetworkTableEntry> shuffleboardFields;

  private final DigitalInput limitSwitch;
  private final TimeOfFlight tof_front;
  public ArrayList<Double> measuredDistances = new ArrayList<>();
  private final DoubleSolenoid dsl_lock;

  private ValueProperty<Boolean> climberActive;
  private double prevMove = 0;

  /**
   * Constructor for the climber.
   */
  public Climber(ValueProperty<Boolean> climberActive) {
    mot_main = new CANSparkMax(Constants.kClimber.CAN_MASTER_MOT, MotorType.kBrushless);
    mot_main.setInverted(false);
    mot_main.setIdleMode(IdleMode.kBrake);

    mot_follower = new CANSparkMax(Constants.kClimber.CAN_FOLLOWER_MOT, MotorType.kBrushless);
    mot_follower.follow(mot_main, true);
    mot_follower.setIdleMode(IdleMode.kBrake);

    // encoder
    encoder_main = mot_main.getEncoder();
    zeroEncoder();

    mot_main.burnFlash();
    mot_follower.burnFlash();

    controller_main = mot_main.getPIDController();
    controller_main.setOutputRange(-1, 1);
    configPID(Constants.kClimber.P, Constants.kClimber.I, Constants.kClimber.D, Constants.kClimber.F);

    locked = false;

    limitSwitch = new DigitalInput(Constants.kClimber.DIGITAL_INPUT_PORT);
    // tof_front = new TimeOfFlight(Constants.kID.ClimberTofMain);
    tof_front = null;

    dsl_lock = new DoubleSolenoid(kID.PneumaticHub, PneumaticsModuleType.REVPH, 14, 15);

    this.climberActive = climberActive;

    // Gives absolute motor positions of 0 - 360 degrees, all positive values.
    // mot_main.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
    shuffleboardFields = new HashMap<String, NetworkTableEntry>();

    ShuffleboardLayout sliders = Shuffleboard.getTab("Climber").getLayout("Auto", BuiltInLayouts.kList);
    ShuffleboardLayout setPoints = Shuffleboard.getTab("Climber").getLayout("Setpoints", BuiltInLayouts.kList);
    ShuffleboardLayout extras = Shuffleboard.getTab("Climber").getLayout("Extras", BuiltInLayouts.kList);

    sliders.add("ELEVATE", new ElevateTo(this)).withWidget(BuiltInWidgets.kCommand);

    setPoints.add("ELEVATE TO MID", new ElevateTo(this, Constants.kClimber.TO_MID_RUNG))
        .withWidget(BuiltInWidgets.kCommand);
    setPoints.add("ELEVATE TO LOW", new ElevateTo(this, Constants.kClimber.TO_LOW_RUNG))
        .withWidget(BuiltInWidgets.kCommand);
    setPoints.add("ELEVATE TO MIN", new ElevateTo(this, Constants.kClimber.TO_MIN_MID)).withWidget(BuiltInWidgets.kCommand);

    shuffleboardFields.put("toPosition", sliders.add("TO POSITION", 0).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.5, "max", 104)).getEntry());

    shuffleboardFields.put("currentPos",
        extras.add("CURRENT POSITION", 0).withWidget(BuiltInWidgets.kTextView).getEntry());

    shuffleboardFields.put("limitSwitch",
        extras.add("LIMIT SWITCH", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry());

    shuffleboardFields.put("idleMode", extras.add("IDLE MODE", "BRAKE").getEntry());

    extras.add("FIND ZERO", new FindElevatorZero(this))
        .withWidget(BuiltInWidgets.kCommand);

    new NetworkButton(extras.add("TOGGLE NEUTRAL MODE", true).withWidget(BuiltInWidgets.kToggleButton).getEntry())
        .whenPressed(new InstantCommand(this::toggleIdleMode));
  }

  public void setIdleMode(IdleMode mode) {
    mot_main.setIdleMode(mode);
    mot_follower.setIdleMode(mode);
  }

  public void toggleIdleMode() {
    String val;

    System.out.println(true);
    if (mot_main.getIdleMode() == IdleMode.kBrake) {
      setIdleMode(IdleMode.kCoast);
      val = "COAST";
    } else {
      setIdleMode(IdleMode.kBrake);

      val = "BRAKE";
    }

    shuffleboardFields.get("idleMode").setString(val);
  }

  public void zeroEncoder() {
    encoder_main.setPosition(0);
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void periodic() {
    shuffleboardFields.get("currentPos").setNumber(getPosition());

    // if (encoder_main.getVelocity() < 0 && encoder_main.getPosition() <=
    // Constants.kClimber.TO_MIN) {
    // disableMotors();
    // } else if (encoder_main.getVelocity() > 0 && encoder_main.getPosition() >=
    // Constants.kClimber.TO_MAX) {
    // disableMotors();
    // }

    shuffleboardFields.get("limitSwitch").setBoolean(getLimitSwitch());
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
    // if (!locked) {
      controller_main.setReference(target, ControlType.kPosition);
    // } else {
    //   disableMotors();
    // }
  }

  public void moveArm(double acceleration, double deceleration) {
    double value = acceleration - deceleration;

    if (value > 0 && getPosition() >= Constants.kClimber.TO_MAX) {
      controller_main.setReference(0, ControlType.kDutyCycle);
      return;
    }

    if (!locked) {
      controller_main.setReference(value, ControlType.kDutyCycle);
    } else {
      disableMotors();
    }
  }

  /**
   * Method for locking the arm.
   */
  public void lockArm() {
    //System.out.println("Locking arm");
    dsl_lock.set(DoubleSolenoid.Value.kForward);
    locked = true;
  }

  /**
   * Method for unlocking the arm.
   */
  public void unlockArm() {
    //System.out.println("Unlocking arm");

    dsl_lock.set(DoubleSolenoid.Value.kReverse);
    locked = false;
  }

  /**
   * This method can be called to toglle the locked value.
   */
  public void toggleLock() {
    locked = !locked;

    if (locked)
      lockArm();
    else
      unlockArm();
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
    // mot_follower.disable();
  }

  public boolean getLimitSwitch() {
    return !limitSwitch.get();
  }

  public void findZero() {
    controller_main.setReference(-0.3, ControlType.kDutyCycle);
  }

  // ---------------------------- Auto Align ---------------------------- //

  /**
   * This method will return the distance read by the Time of Flight sensor.
   * 
   * @return distance in meters
   */
  public double getDistance() {
    return tof_front.getRange() / 1000;
  }

  /**
   * This method will return if the distance read from the Time Of Flight sensor
   * is accurate.
   * 
   * @return valid or invalid.
   */
  public boolean getValidDistance() {
    return tof_front.isRangeValid();
  }

  public void addDistance(double val) {
    measuredDistances.add(val);
  }

  public void clearDistances() {
    measuredDistances.clear();
  }

  public int getNumOfDistances() {
    return measuredDistances.size();
  }

  public double getAvgDistance() {
    //System.out.print("DATA: ");
    //System.out.println(measuredDistances.toString());
    double sum = 0.0;

    double[] arr = new double[measuredDistances.size()];

    for (int i = 0; i < measuredDistances.size(); i++) {
      sum += measuredDistances.get(i);

      arr[i] = measuredDistances.get(i);
    }

    double avg = sum / measuredDistances.size();

    if (Constants.kConfig.DEBUG) {
      SmartDashboard.putNumberArray("Distances", arr);
      SmartDashboard.putNumber("Avergae Distance", avg);
      SmartDashboard.putBoolean("AVG CALLED", true);
    }
    return avg;
  }

  public int getDirection() {
    if (mot_main.getEncoder().getVelocity() > 0)
      return Constants.kClimber.DIRECTION_EXTEND;
    else if (mot_main.getEncoder().getVelocity() < 0)
      return Constants.kClimber.DIRECTION_RETRACT;
    else
      return Constants.kClimber.DIRECTION_STATIONARY;
  }

  public double getRPM() {
    return mot_main.getEncoder().getVelocity();
  }

  public boolean getLocked() {
    return dsl_lock.get() == DoubleSolenoid.Value.kForward;
  }

  public boolean getActive() {
    return climberActive.get();
  }

  public void raiseFrames() {
    MotorUtils.setDefaultStatusPeriod(mot_main);
  }

  public void lowerFrames() {
    MotorUtils.lowerFollowerStatusPeriod(mot_main);
    MotorUtils.lowerFollowerStatusPeriod(mot_follower);
  }

  public void setPrevMove(double move) {
    prevMove = move;
  }

  public double getPrevMove() {
    return prevMove;
  }
}
