package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kID;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  /**
   * Creates a new Pneumatics.
   */
  Compressor compressor1;

  private boolean m_autoFill = true;
  private boolean m_manualAutoFillOverride = false;

  /**
   * Constructor for the Pneumatics class
   */
  public Pneumatics() {
    compressor1 = new Compressor(kID.PneumaticHub, PneumaticsModuleType.REVPH);
    endLoop();
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("PSI", compressor1.getPressure());

    // Check if pressure is too low or too high
    if (compressor1.getPressure() <= Constants.kPneumatics.MIN_PSI) {
      startLoop();
    } else if (compressor1.getPressure() >= Constants.kPneumatics.MAX_PSI) {
      endLoop();
    }

  }

  /**
   * This method will set the closed loop to true.
   */
  private void startLoop() {
    compressor1.enableAnalog(Constants.kPneumatics.MIN_PSI, Constants.kPneumatics.MAX_PSI);
    ;
  }

  /**
   * This method will set the closed loop to be false.
   */
  private void endLoop() {
    // System.out.println(true);
    compressor1.disable();
  }

  /**
   * This method will close the compressor loop control if it is open and open it
   * if it is closed.
   */
  public void toggle() {
    if (compressor1.enabled())
      endLoop();
    else
      startLoop();
  }

  /**
   * This method will close the compressor.
   */
  public void turnOff() {
    compressor1.close();
  }

  /**
   * This method will set the value of m_manualAutoFillOverride
   * 
   * @param state New value of m_manualAutoFillOverride
   */
  public void setManualOverride(boolean state) {
    m_manualAutoFillOverride = state;
  }
}