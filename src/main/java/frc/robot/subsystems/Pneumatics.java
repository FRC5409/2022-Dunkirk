package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsStates;
import frc.robot.Constants.kID;
import frc.robot.Constants.kPneumatics;
import frc.robot.Constants.kConfig;

/**
 * Class for the maintaining the pneumatics subsystem.
 * 
 * @author Selim Abdelwahab, Akil Pathiranage, Lex Stapleton, Sanad Al-Ajrami
 */
public class Pneumatics extends SubsystemBase {

  Compressor compressor1;
  private PneumaticsStates state;

  /**
   * Constructor for the Pneumatics class
   */
  public Pneumatics() {
    compressor1 = new Compressor(kID.PneumaticHub, PneumaticsModuleType.REVPH);

    state = PneumaticsStates.kAny;
    startLoop();
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {
    if(kConfig.DEBUG)
      SmartDashboard.putNumber("PSI", compressor1.getPressure());

    // Check if pressure is too low or too high
    if (state != PneumaticsStates.kStarted && compressor1.getPressure() <= kPneumatics.MIN_PSI) {
      startLoop();
      state = PneumaticsStates.kStarted;
    } else if (state != PneumaticsStates.kEnded && compressor1.getPressure() >= kPneumatics.MAX_PSI) {
      endLoop();
      state = PneumaticsStates.kEnded;
    }

  }

  /**
   * This method will set the closed loop to true.
   */
  private void startLoop() {
    compressor1.enableAnalog(kPneumatics.MIN_PSI, kPneumatics.MAX_PSI);
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
   * This method will close the compressor.
   */
  public void close() {
    compressor1.close();
  }

} 