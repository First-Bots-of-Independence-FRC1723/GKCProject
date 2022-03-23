// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DirectionalDriveCommand extends CommandBase {
  /** Creates a new AutoDriveCommand. */
  DriveSubsystem driveSubsystem;
  Translation2d translation;
  Timer timer;
  double seconds;

  public DirectionalDriveCommand(DriveSubsystem driveSubsystem, Translation2d translation, double seconds) {
    this.driveSubsystem = driveSubsystem;
    this.seconds = seconds;
    timer = new Timer();
    this.translation = translation.times(Constants.Swerve.maxSpeed);

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
    driveSubsystem.drive(translation, 0, false, false);
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
