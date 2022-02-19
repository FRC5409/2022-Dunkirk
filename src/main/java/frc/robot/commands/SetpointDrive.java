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

    private static double getReferenceAngle(double x, double y, double heading){

        if (x == 0 && y == 0){ 
            return heading; 
        }
        else if (y == 0) {
            return -90 * x/Math.abs(x) + 90;
        }
        else if (x == 0) { 
            return -90 * y/Math.abs(y) + 180; 
        }
        
        double ra = Math.abs(Math.toDegrees(Math.atan(y/x)));
 
        if(x < 0 && y > 0) return 180 - ra;
        else if(x < 0 && y < 0) return 180 + ra; 
        else if(x > 0 && y < 0) return 360 - ra; 

        return ra;
    }

    private double GetSetpointHeading(){
        double x = joystick.getRightX();
        double y = joystick.getRightY();

        System.out.print(String.format("x: %s y: %s ", x, y));
        double jason = getReferenceAngle(x, y, gyro.Heading());

        if(jason > gyro.Heading() + 180){
            return gyro.Heading() - gyro.Heading() % + jason;
        }
        else{
            return gyro.Heading() - gyro.Heading() % - jason;
        }
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("init");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double comp = controller.calculate(gyro.Heading());
        double setpoint = GetSetpointHeading();
        System.out.print(String.format("s: %s \n", setpoint));
        controller.setSetpoint(setpoint);
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
        return false;
    }

}