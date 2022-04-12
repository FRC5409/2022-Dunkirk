package frc.robot.configuration;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.base.shooter.ConstantModelProvider;
import frc.robot.base.shooter.ShooterModelProvider;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.ShooterState;

import frc.robot.base.Joystick.ButtonType;

import frc.robot.base.command.ProxySequentialCommandGroup;
import frc.robot.base.command.CommandLock;

import frc.robot.base.indexer.IndexerState;

import frc.robot.base.RobotConfiguration;
import frc.robot.base.CommandProperty;
import frc.robot.base.TriggerProperty;
import frc.robot.base.ValueProperty;
import frc.robot.base.Joystick;
import frc.robot.base.Property;

import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;


import frc.robot.RobotContainer;
import frc.robot.Constants;

import frc.robot.commands.shooter.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.trajectory.ConfigureOdometry;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.FourBallsAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.OneBallAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.ThreeBallsAuto.AltThreeBallsAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.ThreeBallsAuto.SynchronizedThreeBallsAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.TwoBallsAuto.AdvancedTwoBallsAuto;
import frc.robot.commands.autonomous.trajectory.trajectoryAuto.TwoBallsAuto.TwoBallsAuto;

import frc.robot.commands.compound.IndexActive;
import frc.robot.commands.compound.ReverseIndexer;
import frc.robot.commands.compound.SimpleShooter;
import frc.robot.commands.drive.DefaultDrive;

public class RobotCompetition implements RobotConfiguration {
    private final ShooterFlywheel      flywheel;
    private final ShooterTurret        turret;
    private final DriveTrain           drivetrain;
    private final Limelight            limelight;
    private final Indexer              indexer;
    private final Intake               intake;
    private final Climber              climber;

    private final Joystick             joystickPrimary; // = new XboxController(0);
    private final Joystick             joystickSecondary;
    
    private final CommandProperty<IndexerState> indexerState;
    private final CommandProperty<ShooterState> shooterState;
    
    private final TriggerProperty shooterActiveToggle;
    private final TriggerProperty shooterEnabled;
    private final TriggerProperty indexerEnabled;
    private final TriggerProperty climberEnabled;

    private final Property<ShooterConfiguration> shooterConfiguration;
    private final Property<SweepDirection> shooterSweepDirection;
    private final Property<Double> shooterOffset;
    private final Property<Double> drivetrainSpeed;

    private final SendableChooser<Command> autoCommandSelector;

    private final ShooterModelProvider shooterModelProvider;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotCompetition(RobotContainer robot) {
        // Robot Subsystems
        drivetrain        = robot.drivetrain;
        limelight         = robot.limelight;
        flywheel          = robot.flywheel;
        climber           = robot.climber;
        intake            = robot.intake;
        indexer           = robot.indexer;
        turret            = robot.turret;

        // Robot Joysticks
        joystickSecondary = robot.joystickSecondary;
        joystickPrimary   = robot.joystickPrimary;

        // Robot Properties
        shooterSweepDirection = new ValueProperty<>(SweepDirection.kLeft);
        shooterConfiguration  = new ValueProperty<>(Constants.Shooter.CONFIGURATIONS.getConfiguration(ShooterMode.kFar));
        drivetrainSpeed       = new ValueProperty<>(1.0);
        shooterOffset         = new ValueProperty<>(Constants.Shooter.INITIAL_SHOOTER_OFFSET);
        shooterState          = new CommandProperty<>(ShooterState.kOff);
        indexerState          = new CommandProperty<>(IndexerState.kArmed);

        // Robot Trigger Properties
        shooterActiveToggle = new TriggerProperty(false);
        shooterEnabled = new TriggerProperty(true);
        indexerEnabled = new TriggerProperty(true);
        climberEnabled = new TriggerProperty(false);

        // Autonomous Selector
        autoCommandSelector = new SendableChooser<Command>();

        // Shooter Properties
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
        autoCommandSelector.addOption(
            "One",
            new OneBallAuto(drivetrain, indexer, limelight, turret, flywheel, 
                shooterConfiguration, shooterSweepDirection, Property.cast(shooterOffset)));
        
        autoCommandSelector.setDefaultOption(
            "Two",
            new TwoBallsAuto(drivetrain, intake, indexer, limelight, turret, flywheel,
                shooterConfiguration, shooterSweepDirection, Property.cast(shooterOffset)));

        autoCommandSelector.setDefaultOption(
            "Advanced Two",
            new AdvancedTwoBallsAuto(drivetrain, intake, indexer, limelight, turret, flywheel,
                shooterConfiguration, shooterSweepDirection, Property.cast(shooterOffset), shooterState, indexerState));

        autoCommandSelector.addOption(
            "Three",
            new SynchronizedThreeBallsAuto(drivetrain, intake, indexer, limelight, turret, flywheel,
                shooterModelProvider, shooterConfiguration, shooterSweepDirection, indexerState,
                shooterState, shooterOffset));

        autoCommandSelector.addOption(
            "Three Alt",
            new AltThreeBallsAuto(drivetrain, intake, indexer, limelight, turret, flywheel,
                shooterModelProvider, shooterConfiguration, shooterSweepDirection, indexerState,
                shooterState, shooterOffset));

        autoCommandSelector.addOption(
            "Four",
            new FourBallsAuto(drivetrain, intake, indexer, limelight, turret, flywheel,
                shooterModelProvider, shooterConfiguration, shooterSweepDirection, indexerState,
                shooterState, shooterOffset));

        SmartDashboard.putData("Auto Chooser", autoCommandSelector);

        //SmartDashboard.putData("Cycle Limelight View", new ChangeLimelightView(limelight));
        SmartDashboard.putData(
            "Shooter Offset - Increment", 
            new ConfigureProperty<Double>(shooterOffset, () -> shooterOffset.get() + Constants.Shooter.OFFSET_INCREMENT));

        SmartDashboard.putData(
            "Shooter Offset - Decrement", 
            new ConfigureProperty<Double>(shooterOffset, () -> shooterOffset.get() - Constants.Shooter.OFFSET_INCREMENT));
    }

    private void configureCommands() {
        drivetrain.setDefaultCommand(
            new DefaultDrive(drivetrain, joystickPrimary.getController(), drivetrainSpeed));

        climber.setDefaultCommand(
            new DefaultElevator(climber, joystickSecondary.getController()));
    }

    private void configureButtonBindings() {
        CommandLock indexerLock = new CommandLock();
        CommandLock shooterLock = new CommandLock();

        TriggerProperty shooterRunning = new TriggerProperty(false);
        Trigger shooterModeToggle = new Trigger(() -> shooterConfiguration.get().getMode() == ShooterMode.kNear);
        
        Command primeShooter = new PrimeShooter(indexer, indexerState);

        // Cycle drive mode
        joystickPrimary.getButton(ButtonType.kStart)
            .whenActive(drivetrain::cycleDriveMode);

        // Switch drivetrain to fast gear
        joystickPrimary.getButton(ButtonType.kRightBumper)
            .whenActive(new FastGear(drivetrain));
        
        // Switch drivetrain to slow gear
        joystickPrimary.getButton(ButtonType.kRightBumper)
            .whenInactive(new SlowGear(drivetrain));

        // Reverse indexer
        joystickPrimary.getButton(ButtonType.kB)
            .and(indexerEnabled)
            .whileActiveOnce(new ReverseIndexer(indexer, primeShooter, indexerLock, indexerState, shooterState));

        // Run intake and indexer
        joystickPrimary.getButton(ButtonType.kX)
            .and(indexerEnabled)
            .whileActiveOnce(new IndexActive(indexer, intake, joystickPrimary, primeShooter, indexerLock, indexerState, shooterState));
        
        // Prime shooter
        joystickPrimary.getButton(ButtonType.kX)
            .and(indexerEnabled)
            .whenInactive(primeShooter);
            
        // Enter climber mode
        joystickSecondary.getButton(ButtonType.kStart)
            .and(climberEnabled.negate())
            .whenActive(
                new ProxySequentialCommandGroup(
                    new InstantCommand(() -> {
                        // Disable indexer
                        indexerEnabled.set(false);

                        // Disable shooter
                        shooterEnabled.set(false);
                        shooterRunning.set(false);
                        shooterActiveToggle.set(false);

                        // Enable climber
                        climberEnabled.set(true);
                    }),

                    // Configure shooter to near
                    new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kNear),

                    // Interrupt any commands requiring these subsystems
                    new RequireCommand(turret, flywheel, indexer, intake)
                )
            );

        // Exit climber mode
        joystickSecondary.getButton(ButtonType.kStart)
            .and(climberEnabled.negate())
            .whenActive(
                new ProxySequentialCommandGroup(
                    new InstantCommand(() -> {
                        // Enable indexer
                        indexerEnabled.set(true);

                        // Enable shooter
                        shooterEnabled.set(true);
                        shooterRunning.set(false);
                        shooterActiveToggle.set(false);

                        // Disable climber
                        climberEnabled.set(false);
                    }),

                    // Configure shooter to near
                    new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kNear),

                    // Interrupt any commands requiring these subsystems
                    new RequireCommand(turret, flywheel, indexer, intake)
                )
            );

        // Auto align drivetrain (when climbing)
        joystickSecondary.getButton(ButtonType.kX)
            .and(climberEnabled)
            .whenActive(new AutoAlign(climber, drivetrain, 180));

        // Reset gyro (when climbing)
        joystickSecondary.getButton(ButtonType.kB)
            .and(climberEnabled)
            .whenActive(drivetrain::resetGyro);

        // Zero encoder (when climbing)
        joystickSecondary.getButton(ButtonType.kY)
            .and(climberEnabled)
            .whenActive(climber::zeroEncoder);
        
        // Toggle shooter
        joystickSecondary.getButton(ButtonType.kLeftBumper)
            .and(shooterEnabled)
            .and(shooterModeToggle)
            .whenActive(shooterActiveToggle.getProperty().configureTo(() -> !shooterActiveToggle.get()));

        // Run complex shooter
        shooterActiveToggle.whileActiveOnce(
            new ProxySequentialCommandGroup(
                new ConfigureProperty<>(shooterRunning.getProperty(), true),
                
                new ComplexOperateShooter(flywheel, turret, drivetrain, limelight, indexer,
                    new Trigger(joystickSecondary.getController()::getRightBumper),
                    shooterModelProvider, shooterConfiguration, indexerState, shooterSweepDirection, 
                    shooterState, shooterOffset, drivetrainSpeed
                )
            )
        );

        // Zero turret at end of complex shooter
        shooterActiveToggle.negate()
            .and(shooterRunning)
            .whileActiveOnce(
                new SequentialCommandGroup(    
                    new ConfigureProperty<>(shooterRunning.getProperty(), false),
                    new RotateTurret(turret, 0)
                )        
            );

        // Configure shooter to far configuration
        joystickSecondary.getButton(ButtonType.kUpPov)
            .and(shooterEnabled)
            .whenActive(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar));

        // Configure shooter to near configuration
        joystickSecondary.getButton(ButtonType.kDownPov)
            .and(shooterEnabled)
            .whenActive(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kNear));
        
        // Configure shooter sweep direction to left
        joystickSecondary.getButton(ButtonType.kLeftPov)
            .and(shooterEnabled)
            .whenActive(new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kLeft));

        // Configure shooter sweep direction to right
        joystickSecondary.getButton(ButtonType.kRightPov)
            .and(shooterEnabled)
            .whenActive(new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kRight));

        // Run shooter at low configuration
        joystickSecondary.getButton(ButtonType.kA)
            .and(shooterEnabled)
            .whileActiveOnce(
                new SimpleShooter(flywheel, turret, limelight, indexer, shooterLock, 
                    shooterActiveToggle.getProperty(), shooterRunning.getProperty(), 
                    indexerState, shooterState, shooterConfiguration,
                    ShooterMode.kLow, Constants.Shooter.LOW_FLYWHEEL_VELOCITY, 0.5)
            );  
            
        // Run shooter at near configuration
        joystickSecondary.getButton(ButtonType.kRightBumper)
        .and(shooterEnabled)
        .whileActiveOnce(
            new SimpleShooter(flywheel, turret, limelight, indexer, shooterLock, 
                shooterActiveToggle.getProperty(), shooterRunning.getProperty(), 
                indexerState, shooterState, shooterConfiguration,
                ShooterMode.kNear, Constants.Shooter.NEAR_FLYWHEEL_VELOCITY, 0.85)
        );
    }


    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("Shooter Offset", shooterOffset.get());
    }

    /**
     * Method for scheduling commands at the beginning of teleop.
     */
    @Override
    public Command getTeleopCommand() {
        return new ParallelCommandGroup(
            new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar),
            new FindElevatorZero(climber),

            shooterActiveToggle.getProperty().configureTo(false)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public Command getAutonomousCommand() {
        Command command = autoCommandSelector.getSelected();
        if (command == null)
            return null;

        return new SequentialCommandGroup(
            // Enable odometry
            new ConfigureOdometry(drivetrain, true),
            
            // Run autonomous command
            command,

            // Disable odometry
            new ConfigureOdometry(drivetrain, false)
        );
    }
}
