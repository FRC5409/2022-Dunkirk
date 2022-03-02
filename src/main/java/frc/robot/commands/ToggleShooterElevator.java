package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.base.Joystick;
import frc.robot.base.ValueProperty;
import frc.robot.base.Joystick.ButtonType;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.commands.shooter.ConfigureShooter;
import frc.robot.commands.shooter.OperateShooter;
import frc.robot.commands.shooter.RotateTurret;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

public class ToggleShooterElevator extends CommandBase {
    private final Climber climber;
    private final Joystick joystick;
    private final ShooterTurret turret;
    private final Limelight limelight;
    private final ShooterFlywheel flywheel;
    private final Indexer indexer;
    private final DriveTrain drive;

    public ToggleShooterElevator(Joystick joystick, Climber climber, Indexer indexer, ShooterTurret turret, ShooterFlywheel flywheel, Limelight limelight, DriveTrain drive) {
        this.joystick = joystick;
        this.turret = turret;
        this.flywheel = flywheel;
        this.limelight = limelight;
        this.climber = climber;
        this.indexer = indexer;
        this.drive = drive;


        addRequirements(climber, turret, limelight, flywheel);
    }

    @Override
    public void initialize() {
        if (climber.getActive()) {
            ValueProperty<ShooterConfiguration> shooterConfiguration = new ValueProperty<ShooterConfiguration>(Constants.Shooter.CONFIGURATIONS.get(ShooterMode.kFar));
            ValueProperty<SweepDirection> shooterSweepDirection = new ValueProperty<SweepDirection>(SweepDirection.kLeft);

            joystick.getButton(ButtonType.kRightBumper).whileHeld(
                new OperateShooter(limelight, turret, flywheel, indexer, shooterSweepDirection, shooterConfiguration)
            ).whenReleased(new RotateTurret(turret, 0));

            joystick.getButton(ButtonType.kUpPov)
                .and(joystick.getButton(ButtonType.kA).negate())
                .whenActive(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar));

            joystick.getButton(ButtonType.kDownPov)
                .and(joystick.getButton(ButtonType.kA).negate())
                .whenActive(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kNear));

            joystick.getButton(ButtonType.kLeftPov)
                .and(joystick.getButton(ButtonType.kA).negate())
                .whenActive(new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kLeft));

            joystick.getButton(ButtonType.kLeftPov)
                .and(joystick.getButton(ButtonType.kA).negate())
                .whenActive(new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kRight));

            joystick.getButton(ButtonType.kA).whileHeld(new RunShooter(flywheel, indexer, 900));
            
            climber.setActive(false);
            // Shooter activate
        } else {

            joystick.getButton(ButtonType.kX).whenPressed(new AutoAlign(climber, drive, 180));
            joystick.getButton(ButtonType.kB).whenPressed(() -> {
            drive.resetGyro();
            });
            joystick.getButton(ButtonType.kY).whenPressed(() -> {
            climber.zeroEncoder();
            });

            joystick.getButton(ButtonType.kLeftBumper).whenPressed(new FindElevatorZero(climber));



            climber.setActive(true);
            // Shooter deactivate
        }
    }

    private void clearButtons(){

        //TODO remove commands binded to buttons
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
