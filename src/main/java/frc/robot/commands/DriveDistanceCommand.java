// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.Conversions;
import frc.robot.SwerveModule;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends CommandBase {
  /** Creates a new DriveDistanceCommand. */
  private DriveSubsystem driveSubsystem;
  private Translation2d translation;
  private double meters;
  private double cutoffTimeSeconds;
  private Timer timer;

  private double averageEncoderCount;

  public DriveDistanceCommand(DriveSubsystem driveSubsystem, Translation2d translation, double meters, double cutoffTimeSeconds) {
    this.driveSubsystem = driveSubsystem;
    this.translation = translation;
    this.meters = meters;
    this.cutoffTimeSeconds = cutoffTimeSeconds;
    timer = new Timer();

    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // resets encoders
    for(SwerveModule mod : driveSubsystem.mSwerveMods){
      mod.mDriveMotor.setSelectedSensorPosition(0);
    }

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive(translation, 0, false, false);
    
    for(SwerveModule mod : driveSubsystem.mSwerveMods){
      averageEncoderCount += mod.getDriveMotorEncoder();
    }  
    averageEncoderCount /= 4;
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
        ////////// conversions in frc\lib\math\Conversions might low key be wrong, specifically gear ratio stuff
    if(timer.get() > 0){
      return Conversions.falconToMeters(averageEncoderCount) >= meters || timer.get() >= cutoffTimeSeconds;
    }
    return Conversions.falconToMeters(averageEncoderCount) >= meters;
  }
}
