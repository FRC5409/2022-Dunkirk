// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.base.Property;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {
    private final Property<Double> driveSpeed;

    private final XboxController joystick;
    private final DriveTrain drivetrain;


    /**
     * Creates a new DefaultDrive
     *
     * @param drivetrain The drivetrain
     * @param joystick  The input device used by this command.
     */
    public DefaultDrive(DriveTrain drivetrain, XboxController joystick, Property<Double> driveSpeed) {
        this.driveSpeed = driveSpeed;
        this.drivetrain = drivetrain;
        this.joystick = joystick;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        switch (drivetrain.getDriveMode()) {
            case kAadilDrive:
                aadilDriveExecute();
                break;
            case kTankDrive:
                tankDriveExecute();
                break;
            default:
                aadilDriveExecute();
        }
    }

    /**
     * This method runs the robot with the aadilDrive control scheme
     */
    private void aadilDriveExecute() {
        double factor = driveSpeed.get();

        double leftTrigger = joystick.getLeftTriggerAxis() * factor;
        double rightTrigger = joystick.getRightTriggerAxis() * factor;
        double lxAxis = joystick.getLeftX() * factor;

        drivetrain.aadilDrive(leftTrigger, rightTrigger, lxAxis);

    }

    /**
     * This method runs the robot with the tankDrive control scheme
     */
    private void tankDriveExecute() {
        double lyAxis = -joystick.getLeftY();
        double ryAxis = -joystick.getRightY();

        drivetrain.tankDrive(lyAxis, ryAxis);
    }
}
