// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// Subsystems
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

// Commands
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.SimpleDriveAuto;
import frc.robot.commands.FastGear;
import frc.robot.commands.IntakeActive;
import frc.robot.commands.SlowGear;
import frc.robot.commands.TestIndexBelt;
import frc.robot.commands.TestIndexProto;
import frc.robot.commands.TestIndexShoot;
// Misc
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.IntakeIndexGo;
import frc.robot.commands.IntakeSimulationTesting;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ReverseIntakeIndexer;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
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
  private final Pigeon Pigeon;
  //private final Indexer Indexer;
  private final Intake Intake;
  private final Pneumatics pneumatics; 

  // Commands defined
  //private final ExampleCommand m_autoCommand;
  private final DefaultDrive defaultDrive;
  //private final IntakeIndexGo m_intakeIndexGo;
  //private final ReverseIntakeIndexer m_reverseIntakeIndex;
  //private final IntakeSimulationTesting m_intakeSimulationTesting;
  // private final TestIndexBelt m_testIndexBelt;
  // private final TestIndexShoot m_testIndexShoot;
  // private final TestIndexProto m_testIndexProto;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Init controller
    joystick_main = new XboxController(0);

    // Init button binds
    but_main_A = new JoystickButton(joystick_main, XboxController.Button.kA.value);
    but_main_B = new JoystickButton(joystick_main, XboxController.Button.kB.value);
    but_main_X = new JoystickButton(joystick_main, XboxController.Button.kX.value);
    but_main_Y = new JoystickButton(joystick_main, XboxController.Button.kY.value);
    but_main_LBumper = new JoystickButton(joystick_main, XboxController.Button.kLeftBumper.value);
    but_main_RBumper = new JoystickButton(joystick_main, XboxController.Button.kRightBumper.value);
    but_main_LAnalog = new JoystickButton(joystick_main, XboxController.Button.kLeftStick.value);
    but_main_RAnalog = new JoystickButton(joystick_main, XboxController.Button.kRightStick.value);
    but_main_Back = new JoystickButton(joystick_main, XboxController.Button.kBack.value);
    but_main_Start = new JoystickButton(joystick_main, XboxController.Button.kStart.value);

     // Initialize sub systems
     DriveTrain = new DriveTrain();
     Pigeon = new Pigeon();
     //Indexer = new Indexer();
     Intake = new Intake();
     pneumatics = new Pneumatics();

     // Init commands
     defaultDrive = new DefaultDrive((DriveTrain), joystick_main);
    //  m_intakeIndexGo = new IntakeIndexGo(Indexer, Intake);
    //  m_reverseIntakeIndex = new ReverseIntakeIndexer(Intake);
    //  m_intakeSimulationTesting = new IntakeSimulationTesting(Intake);
    //  m_testIndexBelt = new TestIndexBelt(Indexer);
    //  m_testIndexProto = new TestIndexProto(Indexer);
    //  m_testIndexShoot = new TestIndexShoot(Indexer);

 
    // Configure the button bindings
    configureButtonBindings();

    // Sets default command to be DefaultDrive
    DriveTrain.setDefaultCommand(defaultDrive);
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

    // but_main_A.whenPressed();
    but_main_X.whileHeld(new IntakeActive(Intake));
    but_main_B.whileHeld(new ReverseIntake(Intake));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
     
    return new SimpleDriveAuto(DriveTrain);
  }
}
