package frc.robot.commands.autonomous.setPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.Constants.kDriveTrain;

import frc.robot.subsystems.DriveTrain;

public class MoveToAngle extends PIDCommand{
    public DriveTrain drive;

    public MoveToAngle(DriveTrain _drive, double targetAngle) {
        super(
            new PIDController(kDriveTrain.kAngleGains.kP, kDriveTrain.kAngleGains.kI, kDriveTrain.kAngleGains.kD), 
            _drive::Heading, 
            targetAngle, 
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
        System.out.print(output);
        if(output >= 0){
            return Math.min(output,  kDriveTrain.maxTurnSpeed);
        }
        else{
            return Math.max(output, -kDriveTrain.maxTurnSpeed);
        }
    }

    @Override
    public void end(boolean interrupt){
        //drive.setDefaultControlMode();
    }
    
    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return Math.abs(Math.abs(drive.Heading() - getController().getSetpoint()) / getController().getSetpoint()) <= 0.05;
    }
}