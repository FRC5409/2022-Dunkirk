package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.Constants.kDriveTrain;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

public class MoveToDistance extends PIDCommand{
    public MoveToDistance(double targetDistance, DriveTrain drive, Pigeon pigeon) {
        super(
            new PIDController(kDriveTrain.P_Distance, kDriveTrain.I_Distance, kDriveTrain.D_Distance), 
            drive::getEncoderPosition, 
            targetDistance, 
            output -> drive.tankDrive(limitSpeed(output), limitSpeed(output)));

        drive.zeroEncoders();
        addRequirements(drive);
    }
    
    /**
     * @return the double with the smallest magnitude (chooses between the PID output and the maxStraightSpeed)
     * 
     */ 
    private static double limitSpeed(double output){
        if(output >= 0){
            return Math.min(output, kDriveTrain.maxStraightSpeed);
        }
        else{
            return Math.max(output, kDriveTrain.maxStraightSpeed);
        }
    }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
}
