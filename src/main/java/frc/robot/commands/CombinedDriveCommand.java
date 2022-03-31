// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class CombinedDriveCommand extends CommandBase {
  /** Creates a new CombinedDriveCommand. */
  DriveSubsystem driveSubsystem;
  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  private double seconds;
  private Timer timer;

  public CombinedDriveCommand(DriveSubsystem driveSubsystem, Translation2d translation, double rotation, boolean fieldRelative, boolean openLoop, double seconds) {
    this.driveSubsystem = driveSubsystem;
    this.translation = translation;
    this.rotation = rotation;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
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
    driveSubsystem.drive(translation.times(Constants.Swerve.maxSpeed), rotation*Constants.Swerve.maxAngularVelocity, fieldRelative, openLoop);
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
