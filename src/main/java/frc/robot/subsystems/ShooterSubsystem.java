// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

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
  private NetworkTableEntry shooterSpeedEntry = shooterTab.add("shooter speed", 0.29)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .withSize(9, 3)
    .withPosition(2, 1)
    .getEntry();

  public ShooterSubsystem() {}

  // 28% for low from right on the thing
  public void shoot(double multiplier){
    shooterMotorOne.set((shooterSpeedEntry.getDouble(0.0))*(multiplier)*(0.8));
    shooterMotorTwo.set((shooterSpeedEntry.getDouble(0.0))*(multiplier)*(-0.8));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
