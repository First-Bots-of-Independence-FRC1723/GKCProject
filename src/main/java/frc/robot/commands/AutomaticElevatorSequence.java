// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorSequence.AExtendElevator;
import frc.robot.commands.ElevatorSequence.BCloseGrabbyGrabby;
import frc.robot.commands.ElevatorSequence.CPullUp;
import frc.robot.commands.ElevatorSequence.DLetGo;
import frc.robot.commands.ElevatorSequence.EAngleArm;
import frc.robot.commands.ElevatorSequence.FExtendArmAgain;
import frc.robot.commands.ElevatorSequence.GCloseGrabbyGrabbyAgain;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutomaticElevatorSequence extends SequentialCommandGroup {
  /** Creates a new AutomaticElevatorSequence. */
  public AutomaticElevatorSequence(ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AExtendElevator(elevatorSubsystem), 
      new BCloseGrabbyGrabby(elevatorSubsystem), 
      new CPullUp(elevatorSubsystem), 
      new DLetGo(elevatorSubsystem), 
      new EAngleArm(elevatorSubsystem), 
      new FExtendArmAgain(elevatorSubsystem), 
      new GCloseGrabbyGrabbyAgain(elevatorSubsystem));
  }
}
