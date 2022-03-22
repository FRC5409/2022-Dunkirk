package frc.robot.commands.autonomous.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ResetOdometry extends CommandBase{
    final Pose2d m_pose;
    final DriveTrain m_drive;
    
    public ResetOdometry(Pose2d pose, DriveTrain driveTrain){
        m_pose = pose;
        m_drive = driveTrain;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_drive.resetOdometry(m_pose);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
