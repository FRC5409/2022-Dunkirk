// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// Subsystems
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pigeon;

// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.FastGear;
import frc.robot.commands.IntakeActive;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.SetpointDrive;
import frc.robot.commands.SlowGear;

import frc.robot.subsystems.Pneumatics;

import frc.robot.Constants.kAuto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
// Misc
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...

  // Define main joystick
  private final XboxController joystick_main; // = new XboxController(0);
  private final JoystickButton but_main_A, but_main_B, but_main_X, but_main_Y, but_main_LBumper, but_main_RBumper,
      but_main_LAnalog, but_main_RAnalog, but_main_Back, but_main_Start;
  
      
  // Subsystems defined
  private final DriveTrain DriveTrain;
  private final Pneumatics Pneumatics;
  private final Pigeon Pigeon;
  private final Intake intake; 

  // Commands defined
  //private final ExampleCommand m_autoCommand;
  private final DefaultDrive defaultDrive;
  private final SetpointDrive setpointDrive;
  private final IntakeActive intakeActive; 
  private final ReverseIntake reverseIntake; 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Init controller
    joystick_main = new XboxController(0);

    // Init button binds
    but_main_A       = new JoystickButton(joystick_main, XboxController.Button.kA.value);
    but_main_B       = new JoystickButton(joystick_main, XboxController.Button.kB.value);
    but_main_X       = new JoystickButton(joystick_main, XboxController.Button.kX.value);
    but_main_Y       = new JoystickButton(joystick_main, XboxController.Button.kY.value);
    but_main_LBumper = new JoystickButton(joystick_main, XboxController.Button.kLeftBumper.value);
    but_main_RBumper = new JoystickButton(joystick_main, XboxController.Button.kRightBumper.value);
    but_main_LAnalog = new JoystickButton(joystick_main, XboxController.Button.kLeftStick.value);
    but_main_RAnalog = new JoystickButton(joystick_main, XboxController.Button.kRightStick.value);
    but_main_Back    = new JoystickButton(joystick_main, XboxController.Button.kBack.value);
    but_main_Start   = new JoystickButton(joystick_main, XboxController.Button.kStart.value);

     // Initialize sub systems
     DriveTrain = new DriveTrain();
     Pneumatics = new Pneumatics();
     Pigeon     = new Pigeon();
     intake     = new Intake();


     // Init commands
     defaultDrive  = new DefaultDrive(DriveTrain, joystick_main);
     setpointDrive = new SetpointDrive(DriveTrain, Pigeon, joystick_main);
     intakeActive  = new IntakeActive(intake);
     reverseIntake = new ReverseIntake(intake);
 
    // Configure the button bindings
    configureButtonBindings();

    // Sets default command to be DefaultDrive
    DriveTrain.setDefaultCommand(setpointDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Bind start to go to the next drive mode
    but_main_Start.whenPressed(() -> DriveTrain.cycleDriveMode());

    // Bind right bumper to 
    but_main_RBumper.whenPressed(new FastGear(DriveTrain));
    but_main_RBumper.whenReleased( new SlowGear(DriveTrain));

    but_main_X.whileHeld(new IntakeActive(intake));
    but_main_B.whileHeld(new ReverseIntake(intake));
    // but_main_A.whenActive( new MoveToDistance(DriveTrain));
    // but_main_B.toggleWhenPressed( new MoveToAngle(DriveTrain));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
     
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
          new Translation2d(1, 0)
        ), 
        new Pose2d(0, 1, new Rotation2d(0)), 
        config); // new Translation2d(1, 1),  new Translation2d(2, -1)

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

    // returns the autonomous command
    // makes sure that after the auto command is finished running the robot stops.
    return autoCommand.andThen(() -> DriveTrain.tankDriveVolts(0, 0));
  }
}
