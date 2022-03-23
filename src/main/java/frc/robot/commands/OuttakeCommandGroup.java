// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;

public class OuttakeCommandGroup extends ParallelCommandGroup {
  /** Creates a new OuttakeCommandGroup. */
  public OuttakeCommandGroup(ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ShootCommand(shooterSubsystem, false, 0), new TransportCommand(transportSubsystem, false, 0), new IntakeCommand(intakeSubsystem, false, 0));
  }
}