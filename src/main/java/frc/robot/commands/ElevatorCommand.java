// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {
  /** Creates a new ElevatorCommand. */
  private ElevatorSubsystem elevatorSubsystem;
  private boolean up;

  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, boolean up) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.up = up;
    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(up){
      elevatorSubsystem.spool(1);
    } else{
      elevatorSubsystem.spool(-1);
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
