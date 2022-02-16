package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.Constants.kDriveTrain;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

public class MoveToAngle extends PIDCommand{
    public MoveToAngle(double targetDistance, DriveTrain drive, Pigeon pigeon) {
        super(
            new PIDController(kDriveTrain.kAngleGains.kP, kDriveTrain.kAngleGains.kI, kDriveTrain.kAngleGains.kD), 
            pigeon::ContinuousHeading, 
            targetDistance, 
            (output) -> drive.tankDrive(limitSpeed(output), -limitSpeed(output))
              );

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
    public boolean isFinished() {
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
}