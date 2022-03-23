// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class AngleHoodCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  HoodSubsystem hoodSubsystem;
  boolean up;

  public AngleHoodCommand(HoodSubsystem hoodSubsystem, boolean up) {
    this.hoodSubsystem = hoodSubsystem;
    this.up = up;
    addRequirements(hoodSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(up){
      hoodSubsystem.angleHood(-1);
    }
    else{
      hoodSubsystem.angleHood(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.angleHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
