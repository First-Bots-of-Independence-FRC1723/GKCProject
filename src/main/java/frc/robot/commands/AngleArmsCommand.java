// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class AngleArmsCommand extends CommandBase {
  /** Creates a new AngleArmsCommand. */
  ElevatorSubsystem elevatorSubsystem;
  boolean up;

  public AngleArmsCommand(ElevatorSubsystem elevatorSubsystem, boolean up) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.up = up;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(up){
      elevatorSubsystem.angleMotorSimple(1);
    } else{
      elevatorSubsystem.angleMotorSimple(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.angleMotorSimple(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
