// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

import java.util.Objects;

import org.jetbrains.annotations.Nullable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.base.Joystick;
import frc.robot.base.RobotConfiguration;
import frc.robot.base.ValueProperty;
import frc.robot.configuration.RobotCompetition;
import frc.robot.configuration.RobotTraining;
import frc.robot.subsystems.Climber;

public final class RobotContainer {
    public final ValueProperty<Boolean> climberActive;
    public final ShooterFlywheel Flywheel;
    public final ShooterTurret   turret;
    public final DriveTrain      DriveTrain;
    public final Pneumatics      Pneumatics;
    public final Limelight       limelight;
    public final Indexer         Indexer;
    public final Intake          Intake;
    public final Climber         Climber;
    
    public final Joystick        joystickSecondary;
    public final Joystick        joystickPrimary;

    private final RobotConfiguration configuration;
    
    public RobotContainer() {
        climberActive = new ValueProperty<Boolean>(false);

        DriveTrain  = new DriveTrain();
        Pneumatics  = new Pneumatics();
        limelight   = new Limelight();
        Flywheel    = new ShooterFlywheel();
        Climber     = new Climber(climberActive);
        Indexer     = new Indexer();
        Intake      = new Intake();
    //  Pigeon      = new Pigeon();
        turret      = new ShooterTurret();

        joystickPrimary = new Joystick(0);
        joystickSecondary = new Joystick(1);

        configuration = Objects.requireNonNull(getConfiguration(this));
    }

    /**
     * Use this to pass the teleop command to the main {@link Robot} class.
     *
     * @return the command to run in teleop
     */
    public Command getTeleopCommand() {
        return configuration.getTeleopCommand();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return configuration.getAutonomousCommand();
    }
    
    @Nullable
    private static RobotConfiguration getConfiguration(RobotContainer robot) {
        switch (Constants.General.ROBOT_CONFIGURATION) {
            case kCompetition: return new RobotCompetition(robot);
            case kTraining:    return new RobotTraining(robot);
        }

        return null;
    }
}