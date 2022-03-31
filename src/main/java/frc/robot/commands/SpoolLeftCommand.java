// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class SpoolLeftCommand extends CommandBase {
  /** Creates a new SpoolLeftCommand. */
  ElevatorSubsystem elevatorSubsystem;
  boolean positive;

  public SpoolLeftCommand(ElevatorSubsystem elevatorSubsystem, boolean positive) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.positive = positive;

    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("spool left");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(positive){
      elevatorSubsystem.spoolLeft(1);
    } else{
      elevatorSubsystem.spoolLeft(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.spool(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
