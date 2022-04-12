package frc.robot.commands.autonomous.setPoint;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;

import frc.robot.utils.Convert;

public class MoveToDistance extends CommandBase {

    private DriveTrain drive;
    private double setpoint;
    private boolean useSmartDashboard;

    public MoveToDistance(DriveTrain _drive){
        drive = _drive;
        setpoint = 0;
        useSmartDashboard = true;
    }

    public MoveToDistance(DriveTrain _drive, double _setpoint){
        drive = _drive;
        setpoint = _setpoint;
        useSmartDashboard = false;
    }

    @Override
    public void initialize(){
        SmartDashboard.putString("mode", "Position");
        
        if(useSmartDashboard){
            setpoint = SmartDashboard.getNumber("target distance", 0);
        }

        setpoint = Convert.InchesToEncodeUnits(setpoint);

        SmartDashboard.putNumber("setpoint", setpoint);

        drive.zeroEncoders();
        drive.setControlMode(TalonFXControlMode.Position, setpoint);
    }
    

    @Override
    public void end(boolean interrupted){
        drive.stopMotors();
        SmartDashboard.putString("mode", "PercentOutput");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(Math.abs(drive.getEncoderPosition() - setpoint) / setpoint) <= 0.05;
    }
}