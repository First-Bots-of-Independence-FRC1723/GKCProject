// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  IntakeSubsystem intakeSubsystem;
  boolean in;
  double seconds;
  Timer timer;

  public IntakeCommand(IntakeSubsystem intakeSubsystem, boolean in, double seconds) {
    this.intakeSubsystem = intakeSubsystem;
    this.in = in;
    this.seconds = seconds;
    timer = new Timer();

    addRequirements(intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(in){
      intakeSubsystem.intake(1);
    }
    else{
      intakeSubsystem.intake(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.intake(0);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(seconds > 0){
      return timer.get() >= seconds;
    }
    return false;
  }
}
