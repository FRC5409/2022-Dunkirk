package frc.robot.configuration;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.base.shooter.ConstantModelProvider;
//Constants
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.TrackingGains;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;
import frc.robot.base.shooter.target.TargetFiltering;
import frc.robot.base.training.TrainingModel3;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.Joystick.ButtonType;
import frc.robot.base.command.CancelCommand;
import frc.robot.base.command.ProxySequentialCommandGroup;
import frc.robot.base.indexer.IndexerState;
import frc.robot.base.RobotConfiguration;
import frc.robot.base.ValueProperty;
import frc.robot.base.Joystick;
import frc.robot.base.CommandProperty;
// Misc
import edu.wpi.first.wpilibj.XboxController;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight.LedMode;
import frc.robot.commands.indexer.IndexerIntakeActive;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.subsystems.shooter.*;
import frc.robot.training.protocol.NetworkConnection;
import frc.robot.training.protocol.NetworkServerSocket;
import frc.robot.training.protocol.NetworkSocket;
import frc.robot.training.protocol.SendableContext;
import frc.robot.training.protocol.generic.ArraySendable;
import frc.robot.training.protocol.generic.BundleSendable;
import frc.robot.training.protocol.generic.StringSendable;
import frc.robot.training.protocol.generic.ValueSendable;
import frc.robot.utils.Range;
import frc.robot.commands.shooter.*;
// import frc.robot.commands.shooter.experimental.ActiveOperateShooterDelayed;
// import frc.robot.commands.test.ShooterOdometryTracking;
import frc.robot.commands.training.model.ModelTrainingSession;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
// import frc.robot.commands.autonomous.trajectory.trajectoryAuto.OneBallAuto;
// import frc.robot.commands.autonomous.trajectory.trajectoryAuto.TwoBallsAuto;

public class RobotTest implements RobotConfiguration {
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

    private final CommandProperty<IndexerState> indexerArmedState;
    private final CommandProperty<ShooterState> shooterState;
    private final ValueProperty<ShooterConfiguration> shooterConfiguration;
    private final ValueProperty<SweepDirection> shooterSweepDirection;
    private final ValueProperty<Boolean> climberActive;
    private final ValueProperty<Boolean> shooterEnabled;
    private final ValueProperty<Double> shooterOffset;
    private final ValueProperty<Double> drivetrainSpeed;

    private Map<String, TrainingModel3> trainingModels;
    private NetworkConnection clientConnection;

    private ConstantModelProvider shooterModelProvider;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotTest(RobotContainer robot) {
        joystickSecondary = robot.joystickSecondary;
        joystickPrimary   = robot.joystickPrimary;
        DriveTrain        = robot.driveTrain;
        limelight         = robot.limelight;
        Flywheel          = robot.flywheel;
        Climber           = robot.climber;
     // Pigeon            = robot.Pigeon
        Intake            = robot.intake;
        Indexer           = robot.indexer;
        turret            = robot.turret;

        shooterSweepDirection = new ValueProperty<>(SweepDirection.kLeft);
        shooterConfiguration  = new ValueProperty<>(Constants.Shooter.CONFIGURATIONS.getConfiguration(ShooterMode.kFar));
        indexerArmedState     = new CommandProperty<>(IndexerState.kArmed);
        drivetrainSpeed       = new ValueProperty<>(1.0);
        shooterEnabled        = new ValueProperty<>(false);
        climberActive         = robot.climberActive;
        shooterOffset         = new ValueProperty<>(0.0);
        shooterState          = new CommandProperty<>(ShooterState.kOff);

        defaultDrive          = new DefaultDrive(DriveTrain, joystickPrimary.getController(), drivetrainSpeed);
        
        try {
            configureTraining();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        configureButtonBindings();
        configureCommands();
        configureDashboard();
    }

    private void configureTraining() throws IOException {
        SendableContext context = new SendableContext();
            context.registerSendable(ValueSendable.class);
            context.registerSendable(BundleSendable.class);
            context.registerSendable(StringSendable.class);
            context.registerSendable(ArraySendable.class);


        NetworkServerSocket serverSocket = NetworkServerSocket.create(Constants.Training.TRAINER_PORT);
        
        System.out.println("Waiting for connection");
        NetworkSocket clientSocket = serverSocket.accept();

        System.out.println("Established connecton");
        
        clientConnection = new NetworkConnection(clientSocket, context);

        trainingModels = new HashMap<>();
        trainingModels.put("turret_offset", new TrainingModel3(
            0.0, 0.0, 0.0,
            new Range(-3.0, 3.0),
            new Range(-30, 30)
        ));

        trainingModels.put("flywheel_offset", new TrainingModel3(
            0.0, 0.0, 0.0,
            new Range(-3.0, 3.0),
            new Range(-1200, 1200)
        ));

        ShooterTrackingModel trainerTrackingModel = new ShooterTrackingModel(
            TargetFiltering.none(), 
            new TrackingGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            Constants.Vision.TARGET_LOST_TIME,
            
            0.85,
            0.15,

            trainingModels.get("flywheel_offset"),
            trainingModels.get("turret_offset")
        );

        shooterModelProvider = new ConstantModelProvider(
            Constants.Shooter.ODOMETRY_MODEL,
            trainerTrackingModel,
            Constants.Shooter.EXECUTION_MODEL
        );

        SmartDashboard.putData("Tracking Model", trainerTrackingModel);
    }

    private void configureDashboard() {
        Shuffleboard.getTab("Shooter")
            .add("Shooter Offset - Increment", new ConfigureProperty<Double>(shooterOffset, p -> p.set(p.get() + Constants.Shooter.OFFSET_INCREMENT)));

        Shuffleboard.getTab("Shooter")
            .add("Shooter Offset - Decrement", new ConfigureProperty<Double>(shooterOffset, p -> p.set(p.get() - Constants.Shooter.OFFSET_INCREMENT)));

        Shuffleboard.getTab("Testing")
            .add("Limelight - Off", new RunCommand(() -> limelight.setLedMode(LedMode.kModeOn)));

        Shuffleboard.getTab("Testing")
            .add("Limelight - Off", new RunCommand(() -> limelight.setLedMode(LedMode.kModeOff)));

        SmartDashboard.putNumber("Rotation Smoothing", SmartDashboard.getNumber("Rotation Smoothing", 0));
        SmartDashboard.putNumber("Flywheel Offset Factor", SmartDashboard.getNumber("Flywheel Offset Factor", 0));
        SmartDashboard.putNumber("Turret Offset Factor", SmartDashboard.getNumber("Turret Offset Factor", 0));
        SmartDashboard.putNumber("Target Interpolation Factor", SmartDashboard.getNumber("Target Interpolation Factor", 0));
        SmartDashboard.putNumber("Shooter Thresh", SmartDashboard.getNumber("Shooter Thresh", 0));
        SmartDashboard.putNumber("Shooter Proportional", SmartDashboard.getNumber("Shooter Proportional", 0));
        SmartDashboard.putNumber("Simulated Velocity", SmartDashboard.getNumber("Simulated Velocity", 0));
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
                    indexerArmedState.configureTo(IndexerState.kActive),
                    shooterState.notEqualTo(ShooterState.kRun),
                    new RunIndexer(Indexer, -0.5)
                )
            );
        
        Command yes = new PrimeShooter(Indexer, indexerArmedState);

        joystickPrimary.getButton(ButtonType.kX)
            .whileActiveOnce(
                new ProxySequentialCommandGroup(
                    new CancelCommand(yes),
                    indexerArmedState.configureTo(IndexerState.kActive),
                    shooterState.notEqualTo(ShooterState.kRun), 
                    new IndexerIntakeActive(Indexer, Intake, joystickPrimary, joystickSecondary)
                )
            );
        
        joystickPrimary.getButton(ButtonType.kX)
            .whenReleased(yes);

        joystickSecondary.getButton(ButtonType.kStart)
            .whenPressed((new ToggleShooterElevator(climberActive, turret, limelight, Flywheel, Indexer, Climber))
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
                    indexerArmedState.configureTo(IndexerState.kActive),
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
                    indexerArmedState.equalTo(IndexerState.kArmed),
                    new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kLow),
                    new RotateTurret(turret, 0),
                    new RunShooter(Flywheel, Indexer, shooterState, Constants.Shooter.LOW_FLYWHEEL_VELOCITY, 0.5)
                )
            );

        joystickSecondary.getButton(ButtonType.kB)
            .and(climberToggleTrigger.negate())
            .whileActiveOnce(
                new ProxySequentialCommandGroup(
                    indexerArmedState.equalTo(IndexerState.kArmed),
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
            new SequentialCommandGroup(    
                new RotateTurret(turret, 0),
                new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar)
            ),
            
            new ScheduleCommand(
                new ModelTrainingSession(clientConnection, trainingModels)
            ),

            new ConfigureProperty<>(shooterEnabled, false),
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
        return null;
    }
}
