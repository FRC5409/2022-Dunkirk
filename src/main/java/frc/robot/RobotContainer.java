// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Climber;

// Subsystems
import frc.robot.training.BranchType;
import frc.robot.training.Setpoint;
import frc.robot.training.TrainerContext;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.protocol.NetworkClient;
import frc.robot.training.protocol.NetworkSocket;
import frc.robot.training.protocol.SendableContext;
import frc.robot.training.protocol.generic.ArraySendable;
import frc.robot.training.protocol.generic.BundleSendable;
import frc.robot.training.protocol.generic.StringSendable;
import frc.robot.training.protocol.generic.ValueSendable;
// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DisableFlywheel;
import frc.robot.commands.FastGear;
import frc.robot.commands.IndexerIntakeActive;
import frc.robot.commands.IndexerIntakeTest;
import frc.robot.commands.IntakeActive;

import frc.robot.commands.AutoAlign;
import frc.robot.commands.DefaultElevator;
import frc.robot.commands.ElevateTo;
import frc.robot.commands.FindElevatorZero;

import frc.robot.commands.IntakeActive;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.SlowGear;
import frc.robot.commands.shooter.HoodDown;
import frc.robot.commands.shooter.HoodUp;

//Constants
import frc.robot.Constants.kAuto;
import frc.robot.base.Joystick;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.Joystick.ButtonType;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.ShooterModel;
import frc.robot.base.shooter.SweepDirection;

import java.io.IOException;
// Misc
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import frc.robot.commands.IntakeSimulationTesting;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ReverseIntakeIndexer;
import frc.robot.commands.RunIndexerBack;
import frc.robot.commands.ShooterTestOne;
import frc.robot.commands.ShooterTestTwo;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;

import frc.robot.commands.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.training.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Define main joystick
    private final Joystick                 joystick_main;
    private final Joystick                 joystick_secondary;

    // Subsystems defined
    private final DriveTrain               DriveTrain;
    private final Pneumatics               Pneumatics;
    private final Pigeon                   Pigeon;
    private final Indexer                  Indexer;
    private final Intake                   Intake;
    private final ShooterFlywheel          Flywheel;
    private final ShooterTurret            turret;
    private final Limelight                limelight;

    private final DefaultDrive             defaultDrive;
    private final ReverseIntakeIndexer     reverse;
    private final IndexerIntakeActive      indexerIntakeActive;
    private final IntakeActive             intakeActive;
    private final IndexerIntakeTest        test;

    private final TrainerContext           trainerContext;
    private final TrainerDashboard         trainerDashboard;
    private       NetworkClient            trainerClient;

    private final Property<SweepDirection> shooterSweepDirection;
    private final Property<ShooterMode>    shooterMode;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Init controller
        joystick_main = new Joystick(0);
        joystick_secondary = new Joystick(1);

        // Initialize sub systems
        DriveTrain  = new DriveTrain();
        Pneumatics  = new Pneumatics();
        Pigeon      = new Pigeon();
        Intake      = new Intake();
        Indexer     = new Indexer();
        Flywheel    = new ShooterFlywheel();
        turret      = new ShooterTurret();
        limelight   = new Limelight();

        // Init commands
        defaultDrive        = new DefaultDrive((DriveTrain), joystick_main.getController());
        indexerIntakeActive = new IndexerIntakeActive(Indexer, Intake);
        reverse             = new ReverseIntakeIndexer(Intake, Indexer);
        intakeActive        = new IntakeActive(Intake, Indexer);
        test                = new IndexerIntakeTest(Indexer, Intake);

        // Configure the button bindings

        shooterSweepDirection = new ValueProperty<>(SweepDirection.kLeft);
        shooterMode = new ValueProperty<>(ShooterMode.kFar);

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

        configureButtonBindings();

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
        joystick_main.getButton(ButtonType.kStart)
            .whenPressed(() -> DriveTrain.cycleDriveMode());

        // Bind right bumper to
        joystick_main.getButton(ButtonType.kRightBumper)
            .whenPressed(new FastGear(DriveTrain))
            .whenReleased(new SlowGear(DriveTrain));

        // joystick_main.getButton(ButtonType.kA).whenPressed();
        // joystick_main.getButton(ButtonType.kX).whileHeld(new IndexerActive(Indexer, Intake));
        joystick_main.getButton(ButtonType.kY)
            .whileHeld(new IndexerIntakeTest(Indexer, Intake));

        joystick_main.getButton(ButtonType.kB)
            .whileHeld(new ReverseIntakeIndexer(Intake, Indexer));

        joystick_main.getButton(ButtonType.kX)
            .whileHeld(new IndexerIntakeActive(Indexer, Intake));

        joystick_main.getButton(ButtonType.kA)
            .whileHeld(new TrainerRunShooter(limelight, turret, Flywheel, Indexer, trainerDashboard, trainerContext, shooterSweepDirection))
            .whenReleased(new RotateTurret(turret, 0));


        joystick_secondary.getButton(ButtonType.kRightBumper)
            .whileHeld(new ShooterTestTwo(Flywheel, turret, Indexer));

        joystick_secondary.getButton(ButtonType.kLeftBumper)
            .whileHeld(new ShooterTestOne(Flywheel, turret, Indexer));
        
        joystick_secondary.getButton(ButtonType.kX)
            .whenPressed(new BranchTargetSetpoint(trainerDashboard, trainerContext, BranchType.BRANCH_LEFT));

        joystick_secondary.getButton(ButtonType.kB)
            .whenPressed(new BranchTargetSetpoint(trainerDashboard, trainerContext, BranchType.BRANCH_RIGHT));

        joystick_secondary.getButton(ButtonType.kRightBumper)
            .whenPressed(new BranchTargetSetpoint(trainerDashboard, trainerContext, BranchType.BRANCH_CENTER));

        joystick_secondary.getButton(ButtonType.kLeftBumper)
            .whenPressed(new RequestModelUpdate(trainerDashboard, trainerClient, trainerContext));

        joystick_secondary.getButton(ButtonType.kY)
            .whenPressed(new FlipTargetSetpoint(trainerDashboard, trainerContext));
        
        joystick_secondary.getButton(ButtonType.kStart)
            .whenPressed(new SubmitSetpointData(trainerDashboard, trainerClient, trainerContext));

        joystick_secondary.getButton(ButtonType.kLeftStick)
            .whenPressed(new ResetTargetSetpoint(trainerDashboard, trainerContext));

        joystick_secondary.getButton(ButtonType.kA)
            .whileHeld(new TrainerLookShooter(limelight, turret, trainerDashboard, trainerContext, shooterSweepDirection))
            .whenReleased(new RotateTurret(turret, 0));

        joystick_secondary.getButton(ButtonType.kBack)
            .whenPressed(new UndoTargetSetpoint(trainerDashboard, trainerContext));

        joystick_secondary.getButton(ButtonType.kUpPov)
            .and(joystick_secondary.getButton(ButtonType.kA).negate())
            .whenActive(new TrainerConfigureShooterMode(turret, limelight, trainerContext, trainerDashboard, shooterMode, ShooterMode.kFar));

        joystick_secondary.getButton(ButtonType.kDownPov)
            .and(joystick_secondary.getButton(ButtonType.kA).negate())
            .whenActive(new TrainerConfigureShooterMode(turret, limelight, trainerContext, trainerDashboard, shooterMode, ShooterMode.kNear));

        joystick_secondary.getButton(ButtonType.kLeftPov)
            .and(joystick_secondary.getButton(ButtonType.kA).negate())
            .whenActive(new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kLeft));
            
        joystick_secondary.getButton(ButtonType.kRightPov)
            .and(joystick_secondary.getButton(ButtonType.kA).negate())
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