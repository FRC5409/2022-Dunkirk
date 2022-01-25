package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
        setpoint = CaclulateAngle(_setpoint); // calculate distance
        useSmartDashboard = true;
    }

    private static double CaclulateAngle(double angle){
        return Math.toRadians(angle) * Math.PI * Constants.kDriveTrain.wheelSperation;
    }

    @Override
    public void initialize(){
        drive.zeroEncoders(); 

        if(useSmartDashboard){
            if(SmartDashboard.containsKey("target distance")){
                drive.setControlModeSplit(TalonFXControlMode.Position, 
                                          CaclulateAngle(SmartDashboard.getNumber("target angle", 10)), 
                                         -CaclulateAngle(SmartDashboard.getNumber("target angle", 10)));
            }
        }
        else {
            drive.setControlModeSplit(TalonFXControlMode.Position, setpoint, -setpoint);
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
