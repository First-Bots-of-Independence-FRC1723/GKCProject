// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static CTREConfigs ctreConfigs;

  private static ShuffleboardTab bindTab;
  private static NetworkTableEntry intake;
  private static NetworkTableEntry intakePistons;
  private static NetworkTableEntry transport;
  private static NetworkTableEntry shoot;
  private static NetworkTableEntry angleHoodUp;
  private static NetworkTableEntry angleHoodDown;
  private static NetworkTableEntry elevatorUp;
  private static NetworkTableEntry elevatorDown;
  private static NetworkTableEntry autoAim;
  private static NetworkTableEntry evevatorPighstiouxne; 
  private static NetworkTableEntry outtake;
  private static NetworkTableEntry angleArmsForward;
  private static NetworkTableEntry angleArmsBackward;

  public static NetworkTableEntry mod0Offset;
  public static NetworkTableEntry mod1Offset;
  public static NetworkTableEntry mod2Offset;
  public static NetworkTableEntry mod3Offset;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    bindTab = Shuffleboard.getTab("button binds");
    addSwerveEntries();
    addBindEntries();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    setBindEntries();
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void addSwerveEntries(){
    mod0Offset = DriveSubsystem.swerveTab.add("mod0Offset", 46.9).getEntry();
    mod1Offset = DriveSubsystem.swerveTab.add("mod1Offset", 87.7).getEntry();
    mod2Offset = DriveSubsystem.swerveTab.add("mod2Offset", 229.1).getEntry();
    mod3Offset = DriveSubsystem.swerveTab.add("mod3Offset", 169).getEntry();
  }
  
  private void addBindEntries(){
    intake = bindTab.add("intake", "x").getEntry();
    intakePistons = bindTab.add("intake pistons", "left stick button").getEntry();
    transport = bindTab.add("transport", "y").getEntry();
    shoot = bindTab.add("shoot", "a").getEntry();
    angleHoodUp = bindTab.add("angle hood up", "left bumper").getEntry();
    angleHoodDown = bindTab.add("angle hood down", "right bumper").getEntry();
    autoAim = bindTab.add("auto aim" , "b").getEntry();
    outtake = bindTab.add("outtake", "back").getEntry();

    elevatorUp = bindTab.add("elevator up" , "start").getEntry();
    elevatorDown = bindTab.add("elevator down", "back").getEntry();
    angleArmsForward = bindTab.add("angle arms forward", "left bumper").getEntry();
    angleArmsBackward = bindTab.add("angle arms backward", "right bumper").getEntry();
    evevatorPighstiouxne = bindTab.add("evevator piston", "").getEntry();
  }

  private void setBindEntries(){
    intake.setString("x");
    intakePistons.setString("left stick button");
    transport.setString("y");
    shoot.setString("a");
    angleHoodUp.setString("left bumper");
    angleHoodDown.setString("right bumper");
    autoAim.setString("b");
    outtake.setString("back");

    elevatorDown.setString("back");
    elevatorUp.setString("start");
    angleArmsForward.setString("left bumper");
    angleArmsBackward.setString("right bumper");
    evevatorPighstiouxne.setString("a");
    
  }
}

  