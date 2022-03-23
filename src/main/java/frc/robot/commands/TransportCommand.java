// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransportSubsystem;

public class TransportCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  TransportSubsystem transportSubsystem;
  boolean forward;
  double seconds;
  Timer timer;

  public TransportCommand(TransportSubsystem transportSubsystem, boolean forward, double seconds) {
    this.transportSubsystem = transportSubsystem;
    this.forward = forward;
    this.seconds = seconds;
    timer = new Timer();

    addRequirements(transportSubsystem);
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
    if(forward){
      transportSubsystem.transport(-1);
    }
    else{
      transportSubsystem.transport(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transportSubsystem.transport(0);
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
