// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class ReverseIntakeIndexer extends CommandBase {
    private final Intake intake;
    private final Indexer indexer; 

    public ReverseIntakeIndexer(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer; 

        addRequirements(intake, indexer);
    }
    
    @Override
    public void initialize() {
        indexer.enable();
        indexer.setSpeed(-0.5);

        intake.reverseIntake(0.75);
    }

    @Override
    public void end(boolean interuppted) {
        indexer.disable();

        intake.intakeOn(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
