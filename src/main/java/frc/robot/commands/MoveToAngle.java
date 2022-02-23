package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.Constants.kDriveTrain;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

public class MoveToAngle extends PIDCommand{
    public DriveTrain drive;

    public MoveToAngle(double targetDistance, DriveTrain _drive, Pigeon pigeon) {
        super(
            new PIDController(kDriveTrain.kAngleGains.kP, kDriveTrain.kAngleGains.kI, kDriveTrain.kAngleGains.kD), 
            pigeon::Heading, 
            targetDistance, 
            (output) -> _drive.tankDrive(limitSpeed(output), -limitSpeed(output))
              );

        drive = _drive;

        addRequirements(drive);
    }

    /**
     * @return the double with the smallest magnitude (chooses between the PID output and the maxTurnSpeed)
     * 
     */
    private static double limitSpeed(double output){
        if(output >= 0){
            return Math.min(output, -kDriveTrain.maxTurnSpeed);
        }
        else{
            return Math.max(output,  kDriveTrain.maxTurnSpeed);
        }
    }

    @Override
    public void end(boolean interrupt){
        drive.setDefaultControlMode();
    }
    
    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
}