// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private CANSparkMax shooterMotorOne = new CANSparkMax(Constants.Balls.leftShooterMotorPort, MotorType.kBrushless);
  private CANSparkMax shooterMotorTwo = new CANSparkMax(Constants.Balls.rightShooterMotorPort, MotorType.kBrushless);

  private ShuffleboardTab shooterTab = Shuffleboard.getTab("shooter");
  private NetworkTableEntry shooterSpeedEntry = shooterTab.add("shooter speed", 0.5).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

  public ShooterSubsystem() {}

  public void shoot(double multiplier){
    shooterMotorOne.set((shooterSpeedEntry.getDouble(0.0))*(multiplier)*(0.8));
    shooterMotorTwo.set((shooterSpeedEntry.getDouble(0.0))*(multiplier)*(-0.8));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
