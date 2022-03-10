// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Subsystems
// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//Constants
import frc.robot.base.Joystick;
import frc.robot.base.ValueProperty;
import frc.robot.base.Joystick.ButtonType;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.SweepDirection;

// Misc
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;


import frc.robot.commands.*;
import frc.robot.commands.autonomous.trajectoryAuto.OneBallAuto;
import frc.robot.commands.autonomous.trajectoryAuto.ThreeBallsAuto;
import frc.robot.commands.autonomous.trajectoryAuto.TwoBallsAuto;
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
  private final Joystick             joystick_main; // = new XboxController(0);
  private final Joystick             joystick_secondary;

  // Subsystems defined
  private final DriveTrain           DriveTrain;
  private final Pneumatics           Pneumatics;
  // private final Pigeon               Pigeon;
  private final Indexer              Indexer;
  private final Intake               Intake;
  private final ShooterFlywheel      Flywheel;
  private final ShooterTurret        turret;
  private final Limelight            limelight;
  private final Climber Climber;

  private final DefaultDrive         defaultDrive;
  private final ReverseIntakeIndexer reverse;

  private final IndexerIntakeActive  indexerIntakeActive;
  private final IntakeActive         intakeActive;
  private final IndexerIntakeTest    test;

  private final ValueProperty<ShooterConfiguration> shooterConfiguration;
  private final ValueProperty<SweepDirection> shooterSweepDirection;
  private final ValueProperty<Integer> shooterOffset;
  private final ValueProperty<Boolean> climberActive;

  //private final TrainerContext       trainerContext;
  //private final TrainerDashboard     trainerDashboard;
  // private       NetworkClient        trainerClient;

  private final SendableChooser<Command> autoCommandSelector;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Init controller
    joystick_main = new Joystick(0);
    joystick_secondary = new Joystick(1);

    shooterConfiguration = new ValueProperty<>(Constants.Shooter.CONFIGURATIONS.get(ShooterMode.kFar));
    shooterSweepDirection = new ValueProperty<>(SweepDirection.kLeft);
    shooterOffset = new ValueProperty<>(0);
    climberActive = new ValueProperty<Boolean>(false);
    
    // Initialize sub systems

    Climber = new Climber(climberActive);
    DriveTrain  = new DriveTrain();
    Pneumatics  = new Pneumatics();
    // Pigeon      = new Pigeon();
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

    // m_intakeIndexGo = new IntakeIndexGo(Indexer, Intake);
    // m_reverseIntakeIndex = new ReverseIntakeIndexer(Intake);
    // m_intakeSimulationTesting = new IntakeSimulationTesting(Intake);
    // m_testIndexBelt = new TestIndexBelt(Indexer);
    // m_testIndexProto = new TestIndexProto(Indexer);
    // m_testIndexShoot = new TestIndexShoot(Indexer);
    
    // Configure the button bindings


    autoCommandSelector = new SendableChooser<Command>();
    
    autoCommandSelector.setDefaultOption("Default", new TwoBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel, shooterConfiguration, shooterSweepDirection, shooterOffset));
    autoCommandSelector.addOption("One", new OneBallAuto(DriveTrain, Indexer, limelight, turret, Flywheel, shooterConfiguration, shooterSweepDirection, shooterOffset));
    autoCommandSelector.addOption("Two", new TwoBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel, shooterConfiguration, shooterSweepDirection, shooterOffset));
    // autoCommandSelector.addOption("Three", new ThreeBallsAuto(DriveTrain, Intake, Indexer, limelight, turret, Flywheel, shooterConfiguration, shooterSweepDirection, shooterOffset));

    SmartDashboard.putData("Auto Chooser", autoCommandSelector);

    Shuffleboard.getTab("Shooter").add("Hood up", new HoodUp(turret));
    Shuffleboard.getTab("Shooter").add("Hood down", new HoodDown(turret));
    Shuffleboard.getTab("Shooter").add("Shooter Offset - Increment", new ConfigureProperty<Integer>(shooterOffset, p -> p.set(p.get() + Constants.Shooter.OFFSET_INCREMENT)));
    Shuffleboard.getTab("Shooter").add("Shooter Offset - Decrement", new ConfigureProperty<Integer>(shooterOffset, p -> p.set(p.get() - Constants.Shooter.OFFSET_INCREMENT)));
    
    /*
    trainerContext = new TrainerContext(
      new Setpoint(Constants.Training.DISTANCE_RANGE.mid(), Constants.Training.DISTANCE_RANGE),
      new ShooterModel(
        0.0, 0.0, 0.0, 0.0,
        Constants.Training.DISTANCE_RANGE,
        Constants.Shooter.SPEED_RANGE
      )  
    );
    
    trainerDashboard = new TrainerDashboard(trainerContext);
    */
    // try {
    //   configureTraining();
    // } catch (IOException e) {
    //   throw new RuntimeException(e);
    // }

    configureButtonBindings();
    configureCommands();
  }


  private void configureCommands() {
    // Sets default command to be DefaultDrive
    DriveTrain.setDefaultCommand(defaultDrive);
    // Indexer.setDefaultCommand(indexerActive);
    Climber.setDefaultCommand(new DefaultElevator(Climber, joystick_secondary.getController()));
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
    joystick_main.getButton(ButtonType.kRightBumper).whenPressed(new FastGear(DriveTrain));
    joystick_main.getButton(ButtonType.kRightBumper).whenReleased(new SlowGear(DriveTrain));

    // but_main_A.whenPressed();
    joystick_main.getButton(ButtonType.kB).whileHeld(new ReverseIntakeIndexer(Intake, Indexer));
    
    // TODO: temp
    joystick_main.getButton(ButtonType.kX).whileHeld(new IndexerIntakeActive(Indexer, Intake));
    joystick_main.getButton(ButtonType.kX).whenReleased(new RunIndexerBack(Intake, Indexer).withTimeout(0.2));

    Trigger climberToggleTrigger = new Trigger(climberActive::get);
    Trigger shooterModeTrigger = new Trigger(() -> shooterConfiguration.get().getMode() == ShooterMode.kNear);

    joystick_secondary.getButton(ButtonType.kStart)
      .whenPressed((new ToggleShooterElevator(climberActive, turret, limelight, DriveTrain, Flywheel, Indexer, Climber))
      .beforeStarting(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kNear)));

    joystick_secondary.getButton(ButtonType.kX).and(climberToggleTrigger).whenActive(new AutoAlign(Climber, DriveTrain, 180));
    joystick_secondary.getButton(ButtonType.kB).and(climberToggleTrigger).whenActive(() -> {
      DriveTrain.resetGyro();
    });
    joystick_secondary.getButton(ButtonType.kY).and(climberToggleTrigger).whenActive(() -> {
      Climber.zeroEncoder();
    });
    //joystick_secondary.getButton(ButtonType.kLeftBumper).and(climberToggleTrigger).whenActive(new FindElevatorZero(Climber));

    joystick_secondary.getButton(ButtonType.kLeftBumper)
      .and(climberToggleTrigger.negate())
      .and(shooterModeTrigger.negate())
      .whileActiveContinuous(new OperateShooterStaged(
        limelight, turret, Flywheel, Indexer,
        joystick_secondary.getButton(ButtonType.kRightBumper),
        shooterSweepDirection, shooterConfiguration, shooterOffset
      ))
      .whenInactive(new RotateTurret(turret, 0));
      
    // joystick_secondary.getButton(ButtonType.kRightBumper)
    //   .and(climberToggleTrigger.negate())
    //   .and(shooterModeTrigger)
    //   .whileActiveContinuous(new RunShooter(Flywheel, Indexer, Constants.Shooter.NEAR_FLYWHEEL_VELOCITY, 0.85))
    //   .whenInactive(new RotateTurret(turret, 0));

    joystick_secondary.getButton(ButtonType.kUpPov)
        .and(joystick_secondary.getButton(ButtonType.kA).negate()).and(climberToggleTrigger.negate())
        .whenActive(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar));

    joystick_secondary.getButton(ButtonType.kDownPov)
        .and(joystick_secondary.getButton(ButtonType.kA).negate()).and(climberToggleTrigger.negate())
        .whenActive(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kNear));

    joystick_secondary.getButton(ButtonType.kLeftPov)
        .and(joystick_secondary.getButton(ButtonType.kA).negate()).and(climberToggleTrigger.negate())
        .whenActive(new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kLeft));

    joystick_secondary.getButton(ButtonType.kRightPov)
        .and(joystick_secondary.getButton(ButtonType.kA).negate()).and(climberToggleTrigger.negate())
        .whenActive(new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kRight));

    joystick_secondary.getButton(ButtonType.kA)
      .and(climberToggleTrigger.negate())
      .whileActiveContinuous(
        new SequentialCommandGroup(  
          new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kLow),
          new RunShooter(Flywheel, Indexer, Constants.Shooter.LOW_FLYWHEEL_VELOCITY, 0.5)));

          
    joystick_secondary.getButton(ButtonType.kB)
      .and(climberToggleTrigger.negate())
      .whileActiveContinuous(
        new SequentialCommandGroup(  
          new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kGuard),
          new RunShooter(Flywheel, Indexer, Constants.Shooter.GUARD_FLYWHEEL_VELOCITY, 0.5)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   
    return autoCommandSelector.getSelected(); 
  }

  /**
   * Method for scheduling commands at the beginning of teleop.
   */
  public Command getTeleopCommand(){
    return new ParallelCommandGroup(
      new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar),
      new FindElevatorZero(Climber)
    );
  }
}
