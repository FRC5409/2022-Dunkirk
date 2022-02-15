package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.kDriveTrain;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;


public class SetpointDrive extends PIDCommand {

    private final DriveTrain drive;
    private final Pigeon gyro;
    private final XboxController joystick;

    public SetpointDrive(DriveTrain drive, Pigeon gyro, XboxController joystick) {
        super(
            new PIDController(kDriveTrain.kAngleGains.kP, 
                              kDriveTrain.kAngleGains.kI, 
                              kDriveTrain.kAngleGains.kD), 
            gyro::ContinuousHeading, 
            0, 
            output -> drive.aadilDrive(joystick.getRightTriggerAxis(), 
                                       joystick.getLeftTriggerAxis(), 
                                       Math.min(Math.max(output, 1), -1)
                                      )
            );

        this.drive = drive;
        this.gyro = gyro;
        this.joystick = joystick;

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
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        getController().setSetpoint(GetSetpointHeading());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}