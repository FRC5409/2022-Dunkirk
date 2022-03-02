// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Subsystems
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
// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//Constants
import frc.robot.base.Joystick;
import frc.robot.base.Property;
import frc.robot.base.ValueProperty;
import frc.robot.base.Joystick.ButtonType;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterMode;
import frc.robot.base.shooter.ShooterModel;
import frc.robot.base.shooter.SweepDirection;
import frc.robot.base.shooter.VisionPipeline;

import java.io.IOException;
// Misc
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
  private final Joystick             joystick_main; // = new XboxController(0);
  private final Joystick             joystick_secondary;

  // Subsystems defined
  private final DriveTrain           DriveTrain;
  private final Pneumatics           Pneumatics;
  private final Pigeon               Pigeon;
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


  //private final TrainerContext       trainerContext;
  //private final TrainerDashboard     trainerDashboard;
  // private       NetworkClient        trainerClient;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Init controller
    joystick_main = new Joystick(0);
    joystick_secondary = new Joystick(1);

    // Initialize sub systems

    Climber = new Climber();
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

    shooterConfiguration = new ValueProperty<>(Constants.Shooter.CONFIGURATIONS.get(ShooterMode.kFar));
    shooterSweepDirection = new ValueProperty<>(SweepDirection.kLeft);
    shooterOffset = new ValueProperty<>(0);

    // m_intakeIndexGo = new IntakeIndexGo(Indexer, Intake);
    // m_reverseIntakeIndex = new ReverseIntakeIndexer(Intake);
    // m_intakeSimulationTesting = new IntakeSimulationTesting(Intake);
    // m_testIndexBelt = new TestIndexBelt(Indexer);
    // m_testIndexProto = new TestIndexProto(Indexer);
    // m_testIndexShoot = new TestIndexShoot(Indexer);

 
    // but_sec_A = new JoystickButton(joystick_secondary, XboxController.Button.kA.value);
    // but_sec_B = new JoystickButton(joystick_secondary, XboxController.Button.kB.value);
    // but_sec_X = new JoystickButton(joystick_secondary, XboxController.Button.kX.value);
    // but_sec_Y = new JoystickButton(joystick_secondary, XboxController.Button.kY.value);
    // but_sec_LBumper = new JoystickButton(joystick_secondary, XboxController.Button.kLeftBumper.value);
    // but_sec_RBumper = new JoystickButton(joystick_secondary, XboxController.Button.kRightBumper.value);
    // but_sec_LAnalog = new JoystickButton(joystick_secondary, XboxController.Button.kLeftStick.value);
    // but_sec_RAnalog = new JoystickButton(joystick_secondary, XboxController.Button.kRightStick.value);
    // but_sec_Back = new JoystickButton(joystick_secondary, XboxController.Button.kBack.value);
    // but_sec_Start = new JoystickButton(joystick_secondary, XboxController.Button.kStart.value);
    
    // Configure the button bindings

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

    CommandScheduler.getInstance().schedule(new FindElevatorZero(Climber));
    CommandScheduler.getInstance().schedule(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar));
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

    // but_main_A.whenActive( new MoveToDistance(DriveTrain));
    // but_main_B.toggleWhenPressed( new MoveToAngle(DriveTrain));

    // but_sec_Left.whenPressed(new ElevateTo(Climber, true, 0));
    // but_sec_Left.whenPressed(() -> {
    // System.out.println(true);
    // });

    joystick_secondary.getButton(ButtonType.kX).whenPressed(new AutoAlign(Climber, DriveTrain, Pigeon, 180));
    joystick_secondary.getButton(ButtonType.kB).whenPressed(() -> {
      Pigeon.reset();
    });
    joystick_secondary.getButton(ButtonType.kY).whenPressed(() -> {
      Climber.zeroEncoder();
    });

    joystick_secondary.getButton(ButtonType.kLeftBumper).whenPressed(new FindElevatorZero(Climber));

    //joystick_secondary.getButton(ButtonType.kRightBumper).whileHeld(new ShooterTestTwo(Flywheel, turret, Indexer));


    joystick_secondary.getButton(ButtonType.kStart).whenPressed(new ToggleShooterElevator(Climber));

    joystick_secondary.getButton(ButtonType.kRightBumper).whileHeld(
      new OperateShooter(
        limelight, turret, Flywheel, Indexer,
        shooterSweepDirection, shooterConfiguration, shooterOffset)
    ).whenReleased(new RotateTurret(turret, 0));

    joystick_secondary.getButton(ButtonType.kUpPov)
        .and(joystick_secondary.getButton(ButtonType.kA).negate())
        .whenActive(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kFar));

    joystick_secondary.getButton(ButtonType.kDownPov)
        .and(joystick_secondary.getButton(ButtonType.kA).negate())
        .whenActive(new ConfigureShooter(turret, limelight, shooterConfiguration, ShooterMode.kNear));

    joystick_secondary.getButton(ButtonType.kLeftPov)
        .and(joystick_secondary.getButton(ButtonType.kA).negate())
        .whenActive(new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kLeft));

    joystick_secondary.getButton(ButtonType.kLeftPov)
        .and(joystick_secondary.getButton(ButtonType.kA).negate())
        .whenActive(new ConfigureProperty<>(shooterSweepDirection, SweepDirection.kRight));


    //joystick_secondary.getButton(ButtonType.kLeftBumper).whileHeld(new ShooterTestOne(Flywheel, turret, Indexer));
    /*
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

    joystick_main.getButton(ButtonType.kB)
      .whileHeld(new ReverseIntakeIndexer(Intake, Indexer));

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
      .whileHeld(new TrainerLookShooter(limelight, turret, trainerDashboard, trainerContext))
      .whenReleased(new RotateTurret(turret, 0));

      joystick_secondary.getButton(ButtonType.kBack)
        .whenPressed(new UndoTargetSetpoint(trainerDashboard, trainerContext));
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
    }));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /*
    // creates configuration for trajectory
    var feedForward = new SimpleMotorFeedforward(kAuto.ksVolts, kAuto.kvVoltSecondsPerMeter,
        kAuto.kaVoltSecondsSquaredPerMeter);
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(feedForward, kAuto.kDriveKinematics,
        10);

    TrajectoryConfig config = new TrajectoryConfig(kAuto.kMaxSpeed, kAuto.kMaxAcceleration);
    config.setKinematics(kAuto.kDriveKinematics).addConstraint(autoVoltageConstraint);
    // Generates a trajectory that tells the robot to move from its original
    // location
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 0)),
        new Pose2d(0, 1, new Rotation2d(0)),
        config); // new Translation2d(1, 1), new Translation2d(2, -1)

    RamseteCommand autoCommand = new RamseteCommand(trajectory, Pigeon::getPose,
        new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
        new SimpleMotorFeedforward(kAuto.ksVolts, kAuto.kvVoltSecondsPerMeter,
            kAuto.kMaxAcceleration),
        kAuto.kDriveKinematics, DriveTrain::getWheelSpeeds,
        new PIDController(kAuto.kPDriveVel, 0, 0), new PIDController(kAuto.kPDriveVel, 0, 0),
        DriveTrain::tankDriveVolts, DriveTrain);

    // Reset odometry to the starting pose of the trajectory.
    DriveTrain.zeroEncoders();
    Pigeon.resetOdometry(trajectory.getInitialPose());
    
    return null;

    // returns the autonomous command
    // makes sure that after the auto command is finished running the robot stops.
    //return autoCommand.andThen(() -> DriveTrain.tankDriveVolts(0, 0));
    */
    return null; 
  }
}