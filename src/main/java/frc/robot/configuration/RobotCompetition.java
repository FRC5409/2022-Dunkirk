package frc.robot.configuration;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants;
//Constants
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.Joystick.ButtonType;
import frc.robot.base.RobotConfiguration;
import frc.robot.base.ValueProperty;
import frc.robot.base.Joystick;

// Misc
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

import frc.robot.commands.indexer.IndexerIntakeActive;
import frc.robot.commands.indexer.IndexerIntakeTest;
import frc.robot.commands.indexer.IntakeActive;
import frc.robot.commands.indexer.ReverseIntakeIndexer;
import frc.robot.subsystems.shooter.*;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.FourBallsAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.OneBallAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.SynchronizedThreeBallsAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.Test;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.ThreeBallsAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.TwoBallsAuto;

public class RobotCompetition implements RobotConfiguration {
    private final ShooterFlywheel      Flywheel;
    private final ShooterTurret        turret;
    private final DriveTrain           DriveTrain;
    private final Limelight            limelight;
    private final Indexer              Indexer;
    private final Intake               Intake;
    private final Climber              Climber;

    private final Joystick             joystickPrimary; // = new XboxController(0);
    private final Joystick             joystickSecondary;

    private final ReverseIntakeIndexer reverse;
    private final IndexerIntakeActive  indexerIntakeActive;
    private final IndexerIntakeTest    test;
    private final DefaultDrive         defaultDrive;
    private final IntakeActive         intakeActive;

    private final ValueProperty<ShooterConfiguration> shooterConfiguration;
    private final ValueProperty<SweepDirection> shooterSweepDirection;
    private final ValueProperty<Integer> shooterOffset;
    private final ValueProperty<Boolean> climberActive;
    private final ValueProperty<Boolean> shooterArmed;

    private final SendableChooser<Command> autoCommandSelector;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotCompetition(RobotContainer robot) {
        joystickSecondary = robot.joystickSecondary;
        joystickPrimary   = robot.joystickPrimary;
        DriveTrain        = robot.DriveTrain;
        limelight         = robot.limelight;
        Flywheel          = robot.Flywheel;
        Climber           = robot.Climber;
     // Pigeon            = robot.Pigeon
        Intake            = robot.Intake;
        Indexer           = robot.Indexer;
        turret            = robot.turret;

        // Init controller
        shooterSweepDirection = new ValueProperty<>(SweepDirection.kLeft);
        shooterConfiguration  = new ValueProperty<>(Constants.Shooter.CONFIGURATIONS.get(ShooterMode.kFar));
        shooterArmed          = new ValueProperty<>(false);
        shooterOffset         = new ValueProperty<>(0);
        climberActive         = robot.climberActive;
        
        // Initialize sub systems

        // Init commands
        indexerIntakeActive = new IndexerIntakeActive(Indexer, Intake, joystickPrimary, joystickSecondary);
        intakeActive        = new IntakeActive(Intake, Indexer);
        defaultDrive        = new DefaultDrive(DriveTrain, joystickPrimary.getController());
        reverse             = new ReverseIntakeIndexer(Intake, Indexer);
        test                = new IndexerIntakeTest(Indexer, Intake);

        autoCommandSelector = new SendableChooser<Command>();

        configureButtonBindings();
        configureCommands();
        configureDashboard();
    }

    private void configureDashboard() {
        autoCommandSelector.addOption("One",
            new OneBallAuto(DriveTrain, Indexer, limelight, turret, Flywheel, 
                shooterConfiguration, shooterSweepDirection, shooterOffset));
        
        autoCommandSelector.addOption("Two",
            new TwoBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel,
                shooterConfiguration, shooterSweepDirection, shooterOffset));

        autoCommandSelector.addOption("Three",
            new ThreeBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel,
                shooterConfiguration, shooterSweepDirection, shooterOffset));

        autoCommandSelector.setDefaultOption("Three Synchronized",
            new SynchronizedThreeBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel,
                shooterConfiguration, shooterSweepDirection, shooterOffset, shooterArmed));

        autoCommandSelector.addOption("Four",
            new FourBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel,
                shooterConfiguration, shooterSweepDirection, shooterOffset));

        autoCommandSelector.addOption("Test",
            new Test(DriveTrain));

        SmartDashboard.putData("Auto Chooser", autoCommandSelector);

        Shuffleboard.getTab("Shooter")
            .add("Hood up", new HoodUp(turret));

        Shuffleboard.getTab("Shooter")
            .add("Hood down", new HoodDown(turret));

        Shuffleboard.getTab("SmartDashboard")
            .add("Shooter Offset - Increment", new ConfigureProperty<Integer>(shooterOffset, p -> p.set(p.get() + Constants.Shooter.OFFSET_INCREMENT)));

        Shuffleboard.getTab("SmartDashboard")
            .add("Shooter Offset - Decrement", new ConfigureProperty<Integer>(shooterOffset, p -> p.set(p.get() - Constants.Shooter.OFFSET_INCREMENT)));
    }


    private void configureCommands() {
        DriveTrain.setDefaultCommand(defaultDrive);
        Climber.setDefaultCommand(new DefaultElevator(Climber, joystickSecondary.getController()));
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Trigger climberToggleTrigger = new Trigger(climberActive::get);
        Trigger shooterModeTrigger = new Trigger(() -> shooterConfiguration.get().getMode() == ShooterMode.kNear);

        joystickPrimary.getButton(ButtonType.kStart)
            .whenPressed(DriveTrain::cycleDriveMode);

        joystickPrimary.getButton(ButtonType.kRightBumper)
            .whenPressed(new FastGear(DriveTrain));

        joystickPrimary.getButton(ButtonType.kRightBumper)
            .whenReleased(new SlowGear(DriveTrain));

        joystickPrimary.getButton(ButtonType.kB)
            .whileHeld(new ReverseIntakeIndexer(Intake, Indexer));
        
        joystickPrimary.getButton(ButtonType.kX)
            .whileActiveOnce(
                new SequentialCommandGroup(
                    new ConfigureProperty<>(shooterArmed, false),    
                    new IndexerIntakeActive(Indexer, Intake, joystickPrimary, joystickSecondary)
                )
            );
        
        joystickPrimary.getButton(ButtonType.kX)
            .whenReleased(new PrimeShooter(Indexer, shooterArmed).withTimeout(Constants.Shooter.ARMING_TIME));

        joystickSecondary.getButton(ButtonType.kStart)
            .whenPressed((new ToggleShooterElevator(climberActive, turret, limelight, DriveTrain, Flywheel, Indexer, Climber))
            .beforeStarting(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kNear)));

        joystickSecondary.getButton(ButtonType.kX)
            .and(climberToggleTrigger)
            .whenActive(new AutoAlign(Climber, DriveTrain, 180));

        joystickSecondary.getButton(ButtonType.kB)
            .and(climberToggleTrigger)
            .whenActive(DriveTrain::resetGyro);

        joystickSecondary.getButton(ButtonType.kY)
            .and(climberToggleTrigger)
            .whenActive(Climber::zeroEncoder);

        joystickSecondary.getButton(ButtonType.kLeftBumper)
            .and(climberToggleTrigger.negate())
            .and(shooterModeTrigger.negate())
            .whileActiveOnce(
                new OperateShooterDelayed(
                    limelight, turret, Flywheel, Indexer, 
                    shooterSweepDirection, shooterConfiguration, shooterOffset, shooterArmed,
                    new Trigger(joystickSecondary.getController()::getRightBumper)
                )
            )
            .whenInactive(new RotateTurret(turret, 0));
        
        joystickSecondary.getButton(ButtonType.kRightBumper)
            .and(climberToggleTrigger.negate())
            .and(shooterModeTrigger)
            .whileActiveOnce(new RunShooter(Flywheel, Indexer, Constants.Shooter.NEAR_FLYWHEEL_VELOCITY, 0.85))
            .whenInactive(new RotateTurret(turret, 0));

        joystickSecondary.getButton(ButtonType.kUpPov)
            .and(joystickSecondary.getButton(ButtonType.kA).negate())
            .and(climberToggleTrigger.negate())
            .whenActive(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar));

        joystickSecondary.getButton(ButtonType.kDownPov)
            .and(joystickSecondary.getButton(ButtonType.kA).negate())
            .and(climberToggleTrigger.negate())
            .whenActive(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kNear));

        joystickSecondary.getButton(ButtonType.kLeftPov)
            .and(joystickSecondary.getButton(ButtonType.kA).negate())
            .and(climberToggleTrigger.negate())
            .whenActive(new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kLeft));

        joystickSecondary.getButton(ButtonType.kRightPov)
            .and(joystickSecondary.getButton(ButtonType.kA).negate())
            .and(climberToggleTrigger.negate())
            .whenActive(new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kRight));

        joystickSecondary.getButton(ButtonType.kA)
            .and(climberToggleTrigger.negate())
            .whileActiveOnce(
                new SequentialCommandGroup(  
                new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kLow),
                new RunShooter(Flywheel, Indexer, Constants.Shooter.LOW_FLYWHEEL_VELOCITY, 0.5)));

        joystickSecondary.getButton(ButtonType.kB)
            .and(climberToggleTrigger.negate())
            .whileActiveOnce(
                new SequentialCommandGroup(  
                new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kGuard),
                new RunShooter(Flywheel, Indexer, Constants.Shooter.GUARD_FLYWHEEL_VELOCITY, 0.5)));

        
    }

    /**
     * Method for scheduling commands at the beginning of teleop.
     */
    @Override
    public Command getTeleopCommand() {
        return new ParallelCommandGroup(
            new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar),
            new FindElevatorZero(Climber)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public Command getAutonomousCommand() {
        return autoCommandSelector.getSelected(); 
    }
}
