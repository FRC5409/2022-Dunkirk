package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.base.Joystick;
import frc.robot.commands.joystick.JoystickRumble;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.SensorType;

public class IndexerIntakeActive extends CommandBase {
    private final Indexer indexer;
    private final Intake intake;
    private final Command rumbleCommand;

    private boolean tofEnter;
    private boolean active;

    public IndexerIntakeActive(Indexer indexer, Intake intake, Joystick joystickMain, Joystick joystickSecondary) {
        this.indexer = indexer;
        this.intake = intake;

        rumbleCommand = new JoystickRumble(0.5)
            .addJoysticks(joystickMain)
            .withDebounce(0.1)
            .withTimeout(0.5);

        addRequirements(indexer, intake);
    }

    public IndexerIntakeActive(Indexer indexer, Intake intake) {
        this.indexer = indexer;
        this.intake = intake;

        rumbleCommand = null;

        addRequirements(indexer, intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        indexer.enable();
        indexer.setSpeed(0.75);

        // Tested value is 0.3 //Cam bump this up once extra wheels added to first indexer roller
        intake.intakeOn(0.4);
        intake.solenoidsDown();
        

        tofEnter = false;
        active = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean tofEnter = indexer.getSensorState(SensorType.kEnter);
        boolean tofExit = indexer.getSensorState(SensorType.kExit);
        boolean tofBall1 = indexer.getSensorState(SensorType.kBall1);

        if (rumbleCommand != null) {
            if (this.tofEnter != tofEnter && tofEnter)
                rumbleCommand.schedule();
        }
        
        if (tofBall1 && tofExit) {
            if (active) {
                indexer.setSpeed(0);
                active = false;
            }
        } else {
            if (!active) {
                indexer.setSpeed(0.75);
                active = true;
            }
        }

        this.tofEnter = tofEnter;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexer.disable();
        
        intake.intakeOn(0);
        intake.intakeIn(0);
        intake.solenoidsUp();

        if (rumbleCommand != null)
            rumbleCommand.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        // if (TOF_Ball1 && TOF_Ext) {

        //   return true;
        // }

        return false;
    }
}
