package frc.robot.configuration;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.base.shooter.ConstantModelProvider;
//Constants
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.ShooterModelProvider;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.Joystick.ButtonType;
import frc.robot.base.command.CancelCommand;
import frc.robot.base.command.ProxySequentialCommandGroup;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.base.RobotConfiguration;
import frc.robot.base.ValueProperty;
import frc.robot.base.CommandProperty;
import frc.robot.base.Joystick;
import frc.robot.base.Property;
// Misc
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

import frc.robot.commands.indexer.IndexerIntakeActive;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.subsystems.shooter.*;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.FourBallsAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.OneBallAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.ThreeBallsAuto.AltThreeBallsAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.ThreeBallsAuto.SynchronizedThreeBallsAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.TwoBallsAuto.AdvancedTwoBallsAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.TwoBallsAuto.TwoBallsAuto;

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

    private final DefaultDrive         defaultDrive;

    private final CommandProperty<IndexerArmedState> indexerArmedState;
    private final CommandProperty<ShooterState> shooterState;
    private final ValueProperty<ShooterConfiguration> shooterConfiguration;
    private final ValueProperty<SweepDirection> shooterSweepDirection;
    private final ValueProperty<Boolean> climberActive;
    private final ValueProperty<Boolean> shooterEnabled;
    private final ValueProperty<Double> shooterOffset;
    private final ValueProperty<Double> drivetrainSpeed;

    private final SendableChooser<Command> autoCommandSelector;

    private final ShooterModelProvider shooterModelProvider;

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

        shooterSweepDirection = new ValueProperty<>(SweepDirection.kLeft);
        shooterConfiguration  = new ValueProperty<>(Constants.Shooter.CONFIGURATIONS.getConfiguration(ShooterMode.kFar));
        indexerArmedState     = new CommandProperty<>(IndexerArmedState.kArmed);
        drivetrainSpeed       = new ValueProperty<>(1.0);
        shooterEnabled        = new ValueProperty<>(false);
        climberActive         = robot.climberActive;
        shooterOffset         = new ValueProperty<>(Constants.Shooter.INITIAL_SHOOTER_OFFSET);
        shooterState          = new CommandProperty<>(ShooterState.kOff);
        
        defaultDrive        = new DefaultDrive(DriveTrain, joystickPrimary.getController(), drivetrainSpeed);

        autoCommandSelector = new SendableChooser<Command>();

        shooterModelProvider = new ConstantModelProvider(
            Constants.Shooter.ODOMETRY_MODEL,
            Constants.Shooter.TRACKING_MODEL,
            Constants.Shooter.EXECUTION_MODEL
        );

        configureButtonBindings();
        configureDashboard();
        configureCommands();
    }

    private void configureDashboard() {
        autoCommandSelector.addOption("One",
            new OneBallAuto(DriveTrain, Indexer, limelight, turret, Flywheel, 
                shooterConfiguration, shooterSweepDirection, Property.cast(shooterOffset)));
        
        autoCommandSelector.setDefaultOption("Two",
            new TwoBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel,
                shooterConfiguration, shooterSweepDirection, Property.cast(shooterOffset)));

        autoCommandSelector.setDefaultOption("Advanced Two",
            new AdvancedTwoBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel,
                shooterConfiguration, shooterSweepDirection, Property.cast(shooterOffset), shooterState, indexerArmedState));

        autoCommandSelector.addOption("Three",
            new SynchronizedThreeBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel,
                shooterModelProvider, shooterConfiguration, shooterSweepDirection, indexerArmedState,
                shooterState, shooterOffset));

        autoCommandSelector.addOption("Three Alt",
            new AltThreeBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel,
                shooterModelProvider, shooterConfiguration, shooterSweepDirection, indexerArmedState,
                shooterState, shooterOffset));

        autoCommandSelector.addOption("Four",
            new FourBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel,
                shooterModelProvider, shooterConfiguration, shooterSweepDirection, indexerArmedState,
                shooterState, shooterOffset));

        SmartDashboard.putData("Auto Chooser", autoCommandSelector);

        //SmartDashboard.putData("Cycle Limelight View", new ChangeLimelightView(limelight));
        SmartDashboard
            .putData("Shooter Offset - Increment", new ConfigureProperty<Double>(shooterOffset, () -> shooterOffset.get() + Constants.Shooter.OFFSET_INCREMENT));

        SmartDashboard
            .putData("Shooter Offset - Decrement", new ConfigureProperty<Double>(shooterOffset, () -> shooterOffset.get() - Constants.Shooter.OFFSET_INCREMENT));
    }

    private void configureCommands() {
        DriveTrain.setDefaultCommand(defaultDrive);
        Climber.setDefaultCommand(new DefaultElevator(Climber, joystickSecondary.getController()));
    }

    public void teleopPeriodic(){
        SmartDashboard.putNumber("Shooter Offset", shooterOffset.get());
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
        Trigger shooterEnabledTrigger = new Trigger(shooterEnabled::get);
        Trigger climberToggleTrigger = new Trigger(climberActive::get);
        Trigger shooterModeTrigger = new Trigger(() -> shooterConfiguration.get().getMode() == ShooterMode.kNear);
        
        joystickPrimary.getButton(ButtonType.kStart)
            .whenPressed(DriveTrain::cycleDriveMode);

        joystickPrimary.getButton(ButtonType.kRightBumper)
            .whenPressed(new FastGear(DriveTrain));

        joystickPrimary.getButton(ButtonType.kRightBumper)
            .whenReleased(new SlowGear(DriveTrain));

        joystickPrimary.getButton(ButtonType.kB)
            .whileHeld(
                new ProxySequentialCommandGroup(
                    indexerArmedState.configureTo(IndexerArmedState.kActive),
                    shooterState.notEqualTo(ShooterState.kRun),
                    new RunIndexer(Indexer, -0.5)
                )
            );
        
        Command yes = new PrimeShooter(Indexer, indexerArmedState);

        joystickPrimary.getButton(ButtonType.kX)
            .whileActiveOnce(
                new ProxySequentialCommandGroup(
                    new CancelCommand(yes),
                    indexerArmedState.configureTo(IndexerArmedState.kActive),
                    shooterState.notEqualTo(ShooterState.kRun), 
                    new IndexerIntakeActive(Indexer, Intake, joystickPrimary, joystickSecondary)
                )
            );
        
        joystickPrimary.getButton(ButtonType.kX)
            .whenReleased(yes);

        joystickSecondary.getButton(ButtonType.kStart)
            .whenPressed((new ToggleShooterElevator(climberActive, turret, limelight, Flywheel, Indexer, Climber))
            .beforeStarting(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kNear)));

        // joystickSecondary.getButton(ButtonType.kX)
        //     .and(climberToggleTrigger)
        //     .whenActive(new AutoAlign(Climber, DriveTrain, 180));

        joystickSecondary.getButton(ButtonType.kB)
            .and(climberToggleTrigger)
            .whenActive(DriveTrain::resetGyro);

        joystickSecondary.getButton(ButtonType.kY)
            .and(climberToggleTrigger)
            .whenActive(Climber::zeroEncoder);
        
        joystickSecondary.getButton(ButtonType.kLeftBumper)
            .and(climberToggleTrigger.negate())
            .and(shooterModeTrigger.negate())
            .whenActive(new ConfigureProperty<Boolean>(shooterEnabled, () -> !shooterEnabled.get()));

        shooterEnabledTrigger.whileActiveOnce(
            new ComplexOperateShooter(
                Flywheel, turret, DriveTrain, limelight, Indexer,
                new Trigger(joystickSecondary.getController()::getRightBumper),
                shooterModelProvider, shooterConfiguration, indexerArmedState, shooterSweepDirection, 
                shooterState, shooterOffset, drivetrainSpeed
            )
        );

        shooterEnabledTrigger.negate()
            .whileActiveOnce(new RotateTurret(turret, 0));

        joystickSecondary.getButton(ButtonType.kRightBumper)
            .and(climberToggleTrigger.negate())
            .and(shooterModeTrigger)
            .whileActiveOnce(
                new ProxySequentialCommandGroup(
                    indexerArmedState.configureTo(IndexerArmedState.kActive),
                    shooterState.equalTo(ShooterState.kOff), 
                    new RunShooter(Flywheel, Indexer, shooterState, Constants.Shooter.NEAR_FLYWHEEL_VELOCITY, 0.85)
                )
            ).whenInactive(new RotateTurret(turret, 0));
            
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
                new ProxySequentialCommandGroup(
                    indexerArmedState.equalTo(IndexerArmedState.kArmed),
                    new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kLow),
                    new RotateTurret(turret, 0),
                    new RunShooter(Flywheel, Indexer, shooterState, Constants.Shooter.LOW_FLYWHEEL_VELOCITY, 0.5)
                )
            );

        joystickSecondary.getButton(ButtonType.kB)
            .and(climberToggleTrigger.negate())
            .whileActiveOnce(
                new ProxySequentialCommandGroup(
                    indexerArmedState.equalTo(IndexerArmedState.kArmed),
                    new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kGuard),
                    new RotateTurret(turret, 0),
                    new RunShooter(Flywheel, Indexer, shooterState, Constants.Shooter.GUARD_FLYWHEEL_VELOCITY, 0.5)
                )
            );        
    }

    /**
     * Method for scheduling commands at the beginning of teleop.
     */
    @Override
    public Command getTeleopCommand() {
        return new ParallelCommandGroup(
            new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar),
            new FindElevatorZero(Climber),

            new ConfigureProperty<>(shooterEnabled, false)
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
