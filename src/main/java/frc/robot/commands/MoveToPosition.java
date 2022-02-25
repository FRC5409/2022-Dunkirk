package frc.robot.commands;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.kDriveTrain;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;

public class MoveToPosition extends CommandBase {

    private DriveTrain sys_drive;
    private Climber sys_climber;

    private double setpoint;
    private boolean passedSetPoint = false;

    public MoveToPosition(Climber _climber, DriveTrain _drive, double _setpoint) {
        System.out.println("Constructing");
        sys_climber = _climber;
        sys_drive = _drive;
        setpoint = _setpoint;

        addRequirements(_climber, _drive);
        passedSetPoint = true;
    }

    public MoveToPosition(Climber _climber, DriveTrain _drive) {
        System.out.println("Constructing 2");
        sys_climber = _climber;
        sys_drive = _drive;

        addRequirements(_climber, _drive);
        passedSetPoint = false;
    }

    @Override
    public void initialize() {
        sys_drive.zeroEncoders();

        if (!passedSetPoint)
            setpoint = sys_climber.getAvgDistance() - kDriveTrain.DISTANCE_TO_MID_RUN_FROM_WALL;
        
        setpoint = setpoint * Constants.kDriveTrain.METERS_TO_RSU;

        sys_drive.setControlMode(TalonFXControlMode.Position, setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        sys_drive.setControlMode(TalonFXControlMode.PercentOutput, 0);
        sys_drive.zeroEncoders();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // System.out.println(Math.abs(drive.getEncoderPosition() - setpoint) /
        // setpoint);
        return Math.abs(sys_drive.getEncoderPosition()) >= Math.abs(setpoint);
    }
}