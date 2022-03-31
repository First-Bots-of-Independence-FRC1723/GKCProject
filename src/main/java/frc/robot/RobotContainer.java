// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommandGroup;
import frc.robot.commands.SetIntakePistonsNeutral;
import frc.robot.commands.AngleArmsCommand;
import frc.robot.commands.AutoCommandGroup;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpoolLeftCommand;
import frc.robot.commands.SpoolRightCommand;
import frc.robot.commands.ToggleIntakePistonsCommand;
import frc.robot.commands.TransportCommand;
import frc.robot.commands.VariableShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final XboxController driveController = new XboxController(0);
  private final XboxController systemsController = new XboxController(1);
  private final int verticalAxis = XboxController.Axis.kLeftY.value;
  private final int horizontalAxis = XboxController.Axis.kLeftX.value;
  private final int rotationalAxis = XboxController.Axis.kRightX.value;
  private final int rightTriggerAxis = XboxController.Axis.kRightTrigger.value;
  private final int leftTriggerAxis = XboxController.Axis.kLeftTrigger.value;

  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final TransportSubsystem transportSubsystem = new TransportSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    compressor.enableDigital();
    boolean fieldRelative = false;
    boolean openLoop = false;
    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, driveController, verticalAxis, horizontalAxis, rotationalAxis, rightTriggerAxis, leftTriggerAxis, fieldRelative, openLoop));
    shooterSubsystem.setDefaultCommand(new VariableShooterCommand(shooterSubsystem, systemsController));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driveController, Button.kStart.value).whenHeld(new ElevatorCommand(elevatorSubsystem, true));
    new JoystickButton(driveController, Button.kBack.value).whenHeld(new ElevatorCommand(elevatorSubsystem, false));
    new JoystickButton(driveController, Button.kLeftBumper.value).whenHeld(new AngleArmsCommand(elevatorSubsystem, true));
    new JoystickButton(driveController, Button.kRightBumper.value).whenHeld(new AngleArmsCommand(elevatorSubsystem, false));
    new JoystickButton(driveController, Button.kB.value).whenHeld(new SpoolRightCommand(elevatorSubsystem, false));
    new JoystickButton(driveController, Button.kX.value).whenHeld(new SpoolLeftCommand(elevatorSubsystem, false));
    /* 
    new JoystickButton(driveController, Button.kY.value).whenHeld(new DirectionalDriveCommand(driveSubsystem, new Translation2d(0.15, 0), 0));
    new JoystickButton(driveController, Button.kB.value).whenHeld(new DirectionalDriveCommand(driveSubsystem, new Translation2d(0, 0.15), 0));
    new JoystickButton(driveController, Button.kA.value).whenHeld(new DirectionalDriveCommand(driveSubsystem, new Translation2d(-0.15, 0), 0));
    new JoystickButton(driveController, Button.kX.value).whenHeld(new DirectionalDriveCommand(driveSubsystem, new Translation2d(0, -0.15), 0));
    new JoystickButton(driveController, Button.kRightBumper.value).whenHeld(new RotationalDriveCommand(driveSubsystem, 0.3, 0));
    new JoystickButton(driveController, Button.kLeftBumper.value).whenHeld(new RotationalDriveCommand(driveSubsystem, -0.3, 0));
    */

    new JoystickButton(systemsController, Button.kLeftStick.value).whenHeld(new ToggleIntakePistonsCommand(intakeSubsystem));
    new JoystickButton(systemsController, Button.kRightStick.value).whenHeld(new SetIntakePistonsNeutral(intakeSubsystem));
    new JoystickButton(systemsController, Button.kX.value).whenHeld(new IntakeCommand(intakeSubsystem, true, 0));
    new JoystickButton(systemsController, Button.kB.value).whenHeld(new TransportCommand(transportSubsystem, true, 0));
    // new JoystickButton(systemsController, Button.kY.value).whenHeld(new AimCommand(driveSubsystem, hoodSubsystem, 0));
    new JoystickButton(systemsController, Button.kA.value).whenHeld(new ShootCommand(shooterSubsystem, true, 0));
    new JoystickButton(systemsController, Button.kBack.value).whenHeld(new OuttakeCommandGroup(shooterSubsystem, transportSubsystem, intakeSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new AutoCommandGroup(driveSubsystem, hoodSubsystem, shooterSubsystem, transportSubsystem, intakeSubsystem);
  }
}
