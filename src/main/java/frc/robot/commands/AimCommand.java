// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class AimCommand extends CommandBase {
  /** Creates a new AimCommand. */
  DriveSubsystem driveSubsystem;
  HoodSubsystem hoodSubsystem;
  double seconds;
  Timer timer;

  public AimCommand(DriveSubsystem driveSubsystem, HoodSubsystem hoodSubsystem, double seconds) {
    this.driveSubsystem = driveSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.seconds = seconds;
    timer = new Timer();

    addRequirements(driveSubsystem, hoodSubsystem);
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
    Translation2d translation = new Translation2d(0, 0).times(Constants.Swerve.maxSpeed);
    driveSubsystem.drive(translation, hoodSubsystem.tX, false, false);

    hoodSubsystem.aimHoodWithLimelight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
