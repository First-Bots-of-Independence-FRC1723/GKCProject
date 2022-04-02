// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  
  private DriveSubsystem driveSubsystem;
  private XboxController controller;
  private int translationAxis;
  private int strafeAxis;
  private int rotationAxis;
  private int rightTriggerAxis;
  private int leftTriggerAxis;

  public DriveCommand(DriveSubsystem driveSubsystem, XboxController controller, int translationAxis, int strafeAxis, int rotationAxis, int rightTriggerAxis, int leftTriggerAxis, boolean fieldRelative, boolean openLoop) {
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;
    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationAxis = rotationAxis;
    this.rightTriggerAxis = rightTriggerAxis;
    this.leftTriggerAxis = leftTriggerAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;

    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yAxis = -controller.getRawAxis(translationAxis);
    double xAxis = -controller.getRawAxis(strafeAxis);
    double rAxis = -controller.getRawAxis(rotationAxis);

    if(Math.abs(controller.getRawAxis(rightTriggerAxis)) > 0.7){
      yAxis = -controller.getRawAxis(translationAxis);
      xAxis = -controller.getRawAxis(strafeAxis);
      rAxis = -controller.getRawAxis(rotationAxis);
    } else if(Math.abs(controller.getRawAxis(leftTriggerAxis)) > 0.7){
      yAxis = -controller.getRawAxis(translationAxis)*0.1;
      xAxis = -controller.getRawAxis(strafeAxis)*0.1;
      rAxis = -controller.getRawAxis(rotationAxis)*0.1;
    } else{
      yAxis = -controller.getRawAxis(translationAxis)*0.5;
      xAxis = -controller.getRawAxis(strafeAxis)*0.5;
      rAxis = -controller.getRawAxis(rotationAxis)*0.5;
    }

    yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
    rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

    translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
    rotation = rAxis * Constants.Swerve.maxAngularVelocity;
    driveSubsystem.drive(translation, rotation, fieldRelative, openLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
