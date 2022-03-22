package frc.robot.commands.autonomous.setPoint;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kDriveTrain;
import frc.robot.subsystems.DriveTrain;

public class MoveToAngle extends CommandBase {

    private DriveTrain drive;
    private double setpoint;
    private boolean useSmartDashboard;

    public MoveToAngle(DriveTrain _drive){
        drive = _drive;
        setpoint = 0; // calculate distance
        useSmartDashboard = true;
    }

    public MoveToAngle(DriveTrain _drive, double _setpoint){
        drive = _drive;
        setpoint = CalculateDistance(_setpoint); // calculate distance
        useSmartDashboard = true;
    }

    private static double CalculateDistance(double angle){
        return Math.toRadians(angle) * Math.PI * kDriveTrain.wheelSeparation * kDriveTrain.encoderToMeterConversionFactor * 2048;
    }

    @Override
    public void initialize(){
        drive.zeroEncoders(); 

        if(useSmartDashboard){
            if(SmartDashboard.containsKey("target distance")){
                drive.setControlMode(TalonFXControlMode.Position, 
                                          CalculateDistance(SmartDashboard.getNumber("target angle", 10)), 
                                         -CalculateDistance(SmartDashboard.getNumber("target angle", 10)));
            }
        }
        else {
            drive.setControlMode(TalonFXControlMode.Position, setpoint, -setpoint);
        }
    }

    @Override
    public void end(boolean interrupt){
        drive.setControlMode(TalonFXControlMode.PercentOutput, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return drive.getEncoderPositionRight() == setpoint;
    }
}
