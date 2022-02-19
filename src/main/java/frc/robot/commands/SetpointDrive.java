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
            new PIDController(kDriveTrain.kDistanceGains.kP, kDriveTrain.kDistanceGains.kI, kDriveTrain.kDistanceGains.kD), 
            gyro::getAngle, 
            0, 
            output -> drive.aadilDrive(joystick.getRightTriggerAxis(), 
                                       joystick.getLeftTriggerAxis(), 
                                        output)
            );
        // Use addRequirements() here to declare subsystem dependencies.

        this.drive = drive;
        this.gyro = gyro;
        this.joystick = joystick;

        addRequirements(drive);
    }

    private double GetSetpointHeading(){
        double x = joystick.getRightX();
        double y = joystick.getRightY();

        if (x == 0 && y == 0){ return gyro.getAngle(); }
        if (y == 0) {return x;}
        if (x == 0) {return y;}

        double ra = Math.toDegrees(Math.abs(Math.atan(joystick.getRightY() / joystick.getRightX())));

        if(x > 0  && y > 0) { return ra;      }
        if(x < 0  && y > 0) { return 180 - ra;}
        if(x < 0  && y < 0) { return 180 + ra;}
        if(x > 0  && y < 0) {return 360 - ra; }
        else return 0;

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