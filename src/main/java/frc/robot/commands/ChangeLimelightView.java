package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class ChangeLimelightView extends CommandBase {

    private Limelight limelight;

    public ChangeLimelightView(Limelight limelight){
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        limelight.nextViewMode();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
