package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.Constants.kDriveTrain;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

public class TurnToAngle extends PIDCommand{
    public TurnToAngle(double targetDistance, DriveTrain drive, Pigeon pigeon) {
        super(
            new PIDController(kDriveTrain.P_Angle, kDriveTrain.I_Angle, kDriveTrain.D_Angle), 
            pigeon::getAngle, 
            targetDistance, 
            output -> drive.tankDrive(limitSpeed(output), limitSpeed(-output)));

        drive.zeroEncoders();
        addRequirements(drive);
    }

    private static double limitSpeed(double output){
        if(output >= 0){
            return Math.min(output, kDriveTrain.maxTurnSpeed);
        }
        else{
            return Math.max(output, kDriveTrain.maxTurnSpeed);
        }
    }
    
    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
}
