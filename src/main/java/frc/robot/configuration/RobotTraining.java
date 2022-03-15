package frc.robot.configuration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants;
//Constants
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.ShooterModel;
import frc.robot.base.Joystick.ButtonType;
import frc.robot.base.RobotConfiguration;
import frc.robot.base.ValueProperty;
import frc.robot.base.Joystick;
import frc.robot.base.Property;
// Misc
import edu.wpi.first.wpilibj.XboxController;

import java.io.IOException;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.shooter.*;
import frc.robot.training.Setpoint;
import frc.robot.training.SetpointType;
import frc.robot.training.TrainerContext;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.protocol.NetworkClient;
import frc.robot.training.protocol.NetworkSocket;
import frc.robot.training.protocol.SendableContext;
import frc.robot.training.protocol.generic.ArraySendable;
import frc.robot.training.protocol.generic.BundleSendable;
import frc.robot.training.protocol.generic.StringSendable;
import frc.robot.training.protocol.generic.ValueSendable;
import frc.robot.commands.shooter.*;
import frc.robot.commands.training.BranchTargetSetpoint;
import frc.robot.commands.training.FlipTargetSetpoint;
import frc.robot.commands.training.RequestModelUpdate;
import frc.robot.commands.training.ResetTargetSetpoint;
import frc.robot.commands.training.SubmitSetpointData;
import frc.robot.commands.training.TrainerConfigureShooter;
import frc.robot.commands.training.TrainerFocusShooter;
import frc.robot.commands.training.TrainerOperateShooter;
import frc.robot.commands.training.UndoTargetSetpoint;
import frc.robot.commands.training.UpdateTargetSetpoint;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotTraining implements RobotConfiguration {
    private final ShooterFlywheel                Flywheel;
    private final ShooterTurret                  turret;
    private final Pneumatics                     Pneumatics;
    private final DriveTrain                     DriveTrain;
    private final Limelight                      limelight;
    private final Indexer                        Indexer;
    private final Intake                         Intake;
    private final Climber                        Climber;

    private final Joystick                       joystickPrimary; // = new XboxController(0);
    private final Joystick                       joystickSecondary;

    private final ReverseIntakeIndexer           reverse;
    private final IndexerIntakeActive            indexerIntakeActive;
    private final IndexerIntakeTest              test;
    private final DefaultDrive                   defaultDrive;
    private final IntakeActive                   intakeActive;
    
    private final TrainerDashboard               trainerDashboard;
    private       NetworkClient                  trainerClient;
    private       TrainerContext                 trainerContext;
    
    private final Property<ShooterConfiguration> shooterConfiguration;
    private final Property<SweepDirection>       shooterSweepDirection;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotTraining(RobotContainer robot) {
        // Init controller
        joystickPrimary = robot.joystickPrimary;
        joystickSecondary = robot.joystickSecondary;

        // Initialize sub systems
        DriveTrain  = robot.DriveTrain;
        Pneumatics  = robot.Pneumatics;
        limelight   = robot.limelight;
        Flywheel    = robot.Flywheel;
        Climber     = robot.Climber;     
        Indexer     = robot.Indexer;
        Intake      = robot.Intake;
        turret      = robot.turret;

        // Init commands
        defaultDrive        = new DefaultDrive((DriveTrain), joystickPrimary.getController());
        indexerIntakeActive = new IndexerIntakeActive(Indexer, Intake);
        reverse             = new ReverseIntakeIndexer(Intake, Indexer);
        intakeActive        = new IntakeActive(Intake, Indexer);
        test                = new IndexerIntakeTest(Indexer, Intake);

        // Configure the button bindings

        shooterSweepDirection = new ValueProperty<>(SweepDirection.kLeft);
        shooterConfiguration = new ValueProperty<>();

        Shuffleboard.getTab("Turret").add("Hood up", new HoodUp(turret));
        Shuffleboard.getTab("Turret").add("Hood down", new HoodDown(turret));
        
        trainerContext = new TrainerContext(
            new Setpoint(Constants.Training.DISTANCE_RANGE.mid(), Constants.Training.DISTANCE_RANGE)
        );

        trainerContext.setModel(
            ShooterMode.kNear, 
            new ShooterModel(
                0d, 0d, 0d, 0d,
                90.0 - 45.5,
                41.5 / 12.0,
                1d,
                Constants.Shooter.DISTANCE_RANGE,
                Constants.Shooter.SPEED_RANGE
            )
        );
// 80 in
        trainerContext.setModel(
            ShooterMode.kFar, 
            new ShooterModel(
                0d, 0d, 0d, 0d,
                90.0 - 61.5,
                45 / 12.0,
                1d,
                Constants.Shooter.DISTANCE_RANGE,
                Constants.Shooter.SPEED_RANGE
            )
        );

        trainerContext.setMode(ShooterMode.kFar);

        trainerDashboard = new TrainerDashboard(trainerContext);

        try {
            configureTraining();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        configureCommands();
        configureButtonBindings();
    }

    private void configureCommands() {        
        // Sets default command to be DefaultDrive
        DriveTrain.setDefaultCommand(defaultDrive);
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

        // Bind start to go to the next drive mode
        joystickPrimary.getButton(ButtonType.kStart)
            .whenPressed(() -> DriveTrain.cycleDriveMode());

        // Bind right bumper to
        joystickPrimary.getButton(ButtonType.kRightBumper)
            .whenPressed(new FastGear(DriveTrain))
            .whenReleased(new SlowGear(DriveTrain));

        // joystickPrimary.getButton(ButtonType.kA).whenPressed();
        // joystickPrimary.getButton(ButtonType.kX).whileHeld(new IndexerActive(Indexer, Intake));
        joystickPrimary.getButton(ButtonType.kY)
            .whileHeld(new IndexerIntakeTest(Indexer, Intake));

        joystickPrimary.getButton(ButtonType.kB)
            .whileHeld(new ReverseIntakeIndexer(Intake, Indexer));

        joystickPrimary.getButton(ButtonType.kX)
            .whileHeld(new IndexerIntakeActive(Indexer, Intake))
            .whenReleased(new RunIndexerBack(Intake, Indexer).withTimeout(0.2));

        joystickPrimary.getButton(ButtonType.kA)
            .whileHeld(new TrainerOperateShooter(limelight, turret, Flywheel, Indexer, trainerDashboard, trainerContext, shooterSweepDirection))
            .whenReleased(new RotateTurret(turret, 0));

        joystickSecondary.getButton(ButtonType.kRightBumper)
            .whileHeld(new RequestModelUpdate(trainerDashboard, trainerClient, trainerContext));

        // joystickSecondary.getButton(ButtonType.kRightBumper)
        //     .whileHeld(new ShooterTestTwo(Flywheel, turret, Indexer));

        joystickSecondary.getButton(ButtonType.kLeftBumper)
            .whileHeld(new ShooterTestOne(Flywheel, turret, Indexer));
        
        joystickSecondary.getButton(ButtonType.kX)
            .whenPressed(new BranchTargetSetpoint(trainerDashboard, trainerContext, SetpointType.kLeft));

        joystickSecondary.getButton(ButtonType.kB)
            .whenPressed(new BranchTargetSetpoint(trainerDashboard, trainerContext, SetpointType.kRight));

        joystickSecondary.getButton(ButtonType.kRightBumper)
            .whenPressed(new BranchTargetSetpoint(trainerDashboard, trainerContext, SetpointType.kCenter));

        joystickSecondary.getButton(ButtonType.kLeftBumper)
            .whenPressed(new RequestModelUpdate(trainerDashboard, trainerClient, trainerContext));

        joystickSecondary.getButton(ButtonType.kY)
            .whenPressed(new FlipTargetSetpoint(trainerDashboard, trainerContext));
        
        joystickSecondary.getButton(ButtonType.kStart)
            .whenPressed(new SubmitSetpointData(trainerDashboard, trainerClient, trainerContext));

        joystickSecondary.getButton(ButtonType.kLeftStick)
            .whenPressed(new ResetTargetSetpoint(trainerDashboard, trainerContext));

        joystickSecondary.getButton(ButtonType.kRightStick)
            .whenPressed(new UpdateTargetSetpoint(trainerDashboard, trainerContext));

        joystickSecondary.getButton(ButtonType.kA)
            .whileHeld(new TrainerFocusShooter(limelight, turret, trainerDashboard, trainerContext, shooterSweepDirection))
            .whenReleased(new RotateTurret(turret, 0));

        joystickSecondary.getButton(ButtonType.kBack)
            .whenPressed(new UndoTargetSetpoint(trainerDashboard, trainerContext));

        joystickSecondary.getButton(ButtonType.kUpPov)
            .and(joystickSecondary.getButton(ButtonType.kA).negate())
            .whenActive(new TrainerConfigureShooter(turret, limelight, trainerContext, trainerDashboard, shooterConfiguration, ShooterMode.kFar));

        joystickSecondary.getButton(ButtonType.kDownPov)
            .and(joystickSecondary.getButton(ButtonType.kA).negate())
            .whenActive(new TrainerConfigureShooter(turret, limelight, trainerContext, trainerDashboard, shooterConfiguration, ShooterMode.kNear));

        joystickSecondary.getButton(ButtonType.kLeftPov)
            .and(joystickSecondary.getButton(ButtonType.kA).negate())
            .whenActive(new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kLeft));
            
        joystickSecondary.getButton(ButtonType.kRightPov)
            .and(joystickSecondary.getButton(ButtonType.kA).negate())
            .whenActive(new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kRight));
    }  
    
    private void configureTraining() throws IOException {
        SendableContext context = new SendableContext();
            context.registerSendable(StringSendable.class);
            context.registerSendable(ValueSendable.class);
            context.registerSendable(BundleSendable.class);
            context.registerSendable(ArraySendable.class);

        NetworkSocket socket = NetworkSocket.create(Constants.Training.TRAINER_HOSTNAME);
        trainerClient = new NetworkClient(socket, context);

        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            try {
                trainerClient.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null; 
    }
}
