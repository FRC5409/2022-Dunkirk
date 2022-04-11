// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class RunIndexer extends CommandBase {
    private final Indexer indexer; 
    private final double target;

    public RunIndexer(Indexer indexer, double target) {
        this.indexer = indexer;
        this.target = target;

        addRequirements(indexer);
    }
    
    @Override
    public void initialize() {
        indexer.enable();
        indexer.setSpeed(target);
    }

    @Override
    public void end(boolean interuppted) {
        indexer.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
