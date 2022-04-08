package frc.robot.configuration;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.base.shooter.HoodPosition;
//Constants
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.TrackingGains;
import frc.robot.base.shooter.TrainingModelProvider;
import frc.robot.base.shooter.VisionPipeline;
import frc.robot.base.shooter.odometry.ShooterOdometryModel;
import frc.robot.base.shooter.odometry.ShooterTrackingModel;
import frc.robot.base.shooter.target.TargetFiltering;
import frc.robot.base.training.SetpointType;
import frc.robot.base.training.TrainerDashboard;
import frc.robot.base.training.TrainingModel3;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.ShooterTrainingModel4;
import frc.robot.base.Joystick.ButtonType;
import frc.robot.base.command.ProxySequentialCommandGroup;
import frc.robot.base.indexer.IndexerArmedState;
import frc.robot.base.RobotConfiguration;
import frc.robot.base.ValueProperty;
import frc.robot.base.Joystick;
import frc.robot.base.Model3;
import frc.robot.base.Property;
import frc.robot.base.CommandProperty;
// Misc
import edu.wpi.first.wpilibj.XboxController;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight.LedMode;
import frc.robot.commands.indexer.IndexerIntakeActive;
import frc.robot.commands.indexer.IndexerIntakeTest;
import frc.robot.commands.indexer.IntakeActive;
import frc.robot.commands.indexer.ReverseIntakeIndexer;
import frc.robot.commands.indexer.RunIndexerBack;
import frc.robot.subsystems.shooter.*;
import frc.robot.training.protocol.NetworkClient;
import frc.robot.training.protocol.NetworkConnection;
import frc.robot.training.protocol.NetworkServerSocket;
import frc.robot.training.protocol.NetworkSocket;
import frc.robot.training.protocol.SendableContext;
import frc.robot.training.protocol.generic.ArraySendable;
import frc.robot.training.protocol.generic.BundleSendable;
import frc.robot.training.protocol.generic.StringSendable;
import frc.robot.training.protocol.generic.ValueSendable;
import frc.robot.utils.Gains;
import frc.robot.utils.Range;
import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;
import frc.robot.commands.shooter.*;
// import frc.robot.commands.shooter.experimental.ActiveOperateShooterDelayed;
// import frc.robot.commands.test.ShooterOdometryTracking;
import frc.robot.commands.training.model.ModelTrainingSession;
import frc.robot.commands.training.shooter.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
// import frc.robot.commands.autonomous.trajectory.trajectoryAuto.OneBallAuto;
// import frc.robot.commands.autonomous.trajectory.trajectoryAuto.TwoBallsAuto;

public class RobotTraining implements RobotConfiguration {
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
    
    private       TrainerDashboard               trainerDashboard;
    private       NetworkClient                  trainerClient;
    
    private final ValueProperty<ShooterConfiguration> shooterConfiguration;
    private final ValueProperty<SweepDirection> shooterSweepDirection;
    private final ValueProperty<Double> shooterOffset;
    private final ValueProperty<Boolean> climberActive;
    private final ValueProperty<Boolean> shooterArmed;
    private final ValueProperty<Double> drivetrainSpeed;
    private final TrainingModelProvider trainingModelProvider;
    private final Property<Boolean> shooterEnabled;

    private final CommandProperty<ShooterState> shooterState;
    private final CommandProperty<IndexerArmedState> indexerArmedState;

    private final SendableChooser<Command> autoCommandSelector;

    private NetworkConnection clientConnection;
    private Map<String, TrainingModel3> trainingModels;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotTraining(RobotContainer robot) {
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
        shooterConfiguration  = new ValueProperty<>(Constants.Shooter.CONFIGURATIONS.getConfiguration(ShooterMode.kFar));
        shooterArmed          = new ValueProperty<>(false);
        shooterOffset         = new ValueProperty<>(0.0);
        climberActive         = robot.climberActive;
        shooterState          = new CommandProperty<>(ShooterState.kOff);
        indexerArmedState     = new CommandProperty<>(IndexerArmedState.kArmed);
        drivetrainSpeed       = new ValueProperty<>(1.0);
        shooterEnabled        = new ValueProperty<>(false);
        
        // Initialize sub systems

        // Init commands
        indexerIntakeActive = new IndexerIntakeActive(Indexer, Intake, joystickPrimary, joystickSecondary);
        intakeActive        = new IntakeActive(Intake, Indexer);
        defaultDrive        = new DefaultDrive(DriveTrain, joystickPrimary.getController(), drivetrainSpeed);
        reverse             = new ReverseIntakeIndexer(Intake, Indexer);
        test                = new IndexerIntakeTest(Indexer, Intake);
        
        trainingModelProvider = new TrainingModelProvider(
            Constants.Shooter.ODOMETRY_MODEL,
            Constants.Shooter.TRACKING_MODEL,
            new ShooterTrainingModel4(Constants.Shooter.DISTANCE_RANGE, Constants.Shooter.SPEED_RANGE)
        );

        autoCommandSelector = new SendableChooser<Command>();

        try {
            configureTraining();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        configureButtonBindings();
        configureCommands();
        configureDashboard();
    }

    private void configureDashboard() {
        // autoCommandSelector.setDefaultOption("Default", 
        //     new TwoBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel,
        //         shooterConfiguration, shooterSweepDirection, shooterOffset));
        
        // autoCommandSelector.addOption("One",
        //     new OneBallAuto(DriveTrain, Indexer, limelight, turret, Flywheel, 
        //         shooterConfiguration, shooterSweepDirection, shooterOffset));
        
        // autoCommandSelector.addOption("Two",
        //     new TwoBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel,
        //         shooterConfiguration, shooterSweepDirection, shooterOffset));

        SmartDashboard.putData("Auto Chooser", autoCommandSelector);

        Shuffleboard.getTab("Shooter")
            .add("Shooter Offset - Increment", new ConfigureProperty<Double>(shooterOffset, p -> p.set(p.get() + Constants.Shooter.OFFSET_INCREMENT)));

        Shuffleboard.getTab("Shooter")
            .add("Shooter Offset - Decrement", new ConfigureProperty<Double>(shooterOffset, p -> p.set(p.get() - Constants.Shooter.OFFSET_INCREMENT)));
    

        Shuffleboard.getTab("Testing")
            .add("Limelight - On", new CommandBase(){
                public void initialize() {
                    limelight.setLedMode(LedMode.kModeOn);
                };
                public boolean isFinished() { return true; };
            });

            
        Shuffleboard.getTab("Testing")
        .add("Limelight - Off", new CommandBase(){
            public void initialize() {
                limelight.setLedMode(LedMode.kModeOff);
            };
            public boolean isFinished() { return true; };
        });

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

    
    private void configureTraining() throws IOException {
        SendableContext context = new SendableContext();
            context.registerSendable(StringSendable.class);
            context.registerSendable(ValueSendable.class);
            context.registerSendable(BundleSendable.class);
            context.registerSendable(ArraySendable.class);

        NetworkSocket socket = null;
        
        System.out.println("Waiting for connection");
        while (socket == null) {
            try {
                socket = NetworkSocket.create(Constants.Training.TRAINER_HOSTNAME);
            } catch (Exception e) {
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e1) {
                }
            }
        }
            
        trainerClient = new NetworkClient(socket, context);
        trainerDashboard = new TrainerDashboard(trainingModelProvider);

        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            try {
                trainerClient.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }));
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
            .whileHeld(
                new ProxySequentialCommandGroup(
                    indexerArmedState.configureTo(IndexerArmedState.kActive),
                    shooterState.notEqualTo(ShooterState.kRun),
                    new ReverseIntakeIndexer(Intake, Indexer)
                )
            );
        
        joystickPrimary.getButton(ButtonType.kX)
            .whileActiveOnce(
                new ProxySequentialCommandGroup(
                    indexerArmedState.configureTo(IndexerArmedState.kActive),
                    shooterState.notEqualTo(ShooterState.kRun), 
                    new IndexerIntakeActive(Indexer, Intake, joystickPrimary, joystickSecondary)
                )
            );
        
        joystickPrimary.getButton(ButtonType.kX)
            .whenReleased(new PrimeShooter(Indexer, indexerArmedState).withTimeout(Constants.Shooter.ARMING_TIME));

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

        Trigger shooterTrigger = new Trigger(shooterEnabled::get);

        ComplexOperateShooter shooterCommand = new ComplexOperateShooter(
            Flywheel, turret, DriveTrain, limelight, Indexer,
            new Trigger(joystickSecondary.getController()::getRightBumper),
            trainingModelProvider, shooterConfiguration, indexerArmedState, shooterSweepDirection, shooterState, 
            shooterOffset, drivetrainSpeed
        );

        shooterTrigger.whileActiveOnce(shooterCommand);
        shooterTrigger.negate()
            .whileActiveOnce(new RotateTurret(turret, 0));

        SmartDashboard.putData("Shooter State Machine", shooterCommand);

        // yes.negate().toggleWhenActive(
        //     new RotateTurret(turret, 0)
        // );

    // joystickSecondary.getButton(ButtonType.kRightBumper)
    //     .whileHeld(new ShooterTestTwo(Flywheel, turret, Indexer));

    //joystickSecondary.getButton(ButtonType.kLeftBumper)
    //    .whileHeld(new ShooterTestOne(Flywheel, turret, Indexer));
    
    joystickSecondary.getButton(ButtonType.kX)
        .whenPressed(new BranchTargetSetpoint(trainingModelProvider, trainerDashboard, SetpointType.kLeft));

    joystickSecondary.getButton(ButtonType.kB)
        .whenPressed(new BranchTargetSetpoint(trainingModelProvider, trainerDashboard, SetpointType.kRight));

    joystickSecondary.getButton(ButtonType.kRightTrigger)
        .whenPressed(new BranchTargetSetpoint(trainingModelProvider, trainerDashboard, SetpointType.kCenter));

    joystickSecondary.getButton(ButtonType.kLeftTrigger)
        .whenPressed(new RequestModelUpdate(shooterConfiguration, trainingModelProvider, trainerDashboard, trainerClient));

    joystickSecondary.getButton(ButtonType.kY)
        .whenPressed(new FlipTargetSetpoint(trainingModelProvider, trainerDashboard));
    
    joystickSecondary.getButton(ButtonType.kStart)
        .whenPressed(new SubmitSetpointData(shooterConfiguration, trainingModelProvider, trainerDashboard, trainerClient));

    joystickSecondary.getButton(ButtonType.kLeftStick)
        .whenPressed(new ResetTargetSetpoint(trainingModelProvider, trainerDashboard));

    joystickSecondary.getButton(ButtonType.kRightStick)
        .whenPressed(new UpdateTargetSetpoint(trainingModelProvider, trainerDashboard));

    // joystickPrimary.getButton(ButtonType.kA)
    //     .whileHeld(new TrainerFocusShooter(trainingModelProvider, trainerDashboard, turret, limelight, shooterSweepDirection))
    //     .whenReleased(new RotateTurret(turret, 0));

    joystickSecondary.getButton(ButtonType.kBack)
        .whenPressed(new UndoTargetSetpoint(trainingModelProvider, trainerDashboard));

    joystickSecondary.getButton(ButtonType.kUpPov)
        .and(joystickSecondary.getButton(ButtonType.kA).negate())
        .whenActive(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar));

    joystickSecondary.getButton(ButtonType.kDownPov)
        .and(joystickSecondary.getButton(ButtonType.kA).negate())
        .whenActive(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kNear));
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
            new FindElevatorZero(Climber),

            new ConfigureProperty<>(shooterEnabled, false)
            // new ScheduleCommand(
            //     new ModelTrainingSession(clientConnection, trainingModels),
            //     new CommandBase() {
            //         public void execute() {
            //             SmartDashboard.putString("Shooter State", shooterState.toString());
            //             SmartDashboard.putString("Indexer Armed State", indexerArmedState.toString());
            //         };

            //         @Override
            //         public boolean isFinished() {
            //             return false;
            //         }
            //     }
            // )
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
