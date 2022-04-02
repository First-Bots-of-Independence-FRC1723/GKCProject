// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAngle. */
  DriveSubsystem driveSubsystem;
  private double angle;
  private double tolerance;
  private double seconds;
  private Timer timer;

  public TurnToAngle(DriveSubsystem driveSubsystem, double angle, double tolerance, double seconds) {
    this.driveSubsystem = driveSubsystem;
    this.angle = angle;
    this.tolerance = tolerance;
    this.seconds = seconds;
    timer = new Timer();

    addRequirements(driveSubsystem);
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
    if(driveSubsystem.getYaw().getDegrees() >= angle+tolerance){
      driveSubsystem.drive(new Translation2d(0, 0), 2, false, false);
    } else if(driveSubsystem.getYaw().getDegrees() <= angle-tolerance){
      driveSubsystem.drive(new Translation2d(0, 0), -2, false, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new Translation2d(0, 0), 0, false, false);
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
