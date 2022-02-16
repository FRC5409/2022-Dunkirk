package frc.robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.kDriveTrain;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

import frc.robot.utils.PIDController;


public class SetpointDrive extends CommandBase {

    private final DriveTrain drive;
    private final Pigeon gyro;
    private final XboxController joystick;

    private final PIDController controller;

    public SetpointDrive(DriveTrain _drive, Pigeon _gyro, XboxController _joystick) {
                                                                 
        controller = new PIDController(kDriveTrain.kAngleGains.kP, kDriveTrain.kAngleGains.kI, kDriveTrain.kAngleGains.kD);

        drive = _drive;
        gyro = _gyro;
        joystick = _joystick;

        System.out.println("Setpoint drive contr");

        addRequirements(drive);
    }

    private double GetSetpointHeading(){
        double x = joystick.getRightX();
        double y = joystick.getRightY();

        if (x == 0 && y == 0){ 
            return gyro.ContinuousHeading(); 
        }
        else if (y == 0) {
            return gyro.ContinuousHeading() + Math.toDegrees(x/Math.abs(x)) - gyro.ContinuousHeading() % 360; 
        }
        else if (x == 0) { 
            return gyro.ContinuousHeading() + Math.toDegrees(y/Math.abs(y)) - gyro.ContinuousHeading() % 360; 
        }
        else {
            return gyro.ContinuousHeading() + Math.toDegrees(Math.atan(joystick.getRightY() / joystick.getRightX())) - gyro.ContinuousHeading() % 360;
        }

        /*
        if(x < 0  && y > 0) { ra = 180 - ra; }
        else if(x < 0  && y < 0) { ra = 180 + ra; }
        else if(x > 0  && y < 0) { ra = 360 - ra; }

        ra = ra % 360 + (int)(gyro.ContinuousHeading() / 360);
        */
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("init");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("execute");
        double comp = controller.calculate(gyro.ContinuousHeading());
        controller.setSetpoint(GetSetpointHeading());

        drive.aadilDrive(joystick.getRightTriggerAxis(), 
                         joystick.getLeftTriggerAxis(), 
                         Math.min(Math.max(comp, 1), -1)
                         );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        
        System.out.println("finished");
        return false;
    }

}