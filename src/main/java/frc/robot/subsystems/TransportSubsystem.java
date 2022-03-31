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

public class TransportSubsystem extends SubsystemBase {
  /** Creates a new TransportSubsystem. */
  private CANSparkMax transportMotor = new CANSparkMax(Constants.Balls.transportMotorPort, MotorType.kBrushless);

  private ShuffleboardTab transportTab = Shuffleboard.getTab("transport");
  private NetworkTableEntry transportSpeedEntry = transportTab.add("transport speed", 0.5)
  .withWidget(BuiltInWidgets.kNumberSlider)
  .withProperties(Map.of("min", 0, "max", 1))
  .withSize(9, 3)
  .withPosition(2, 1)
  .getEntry();

  public TransportSubsystem() {}

  // ~50%
  public void transport(double multiplier){
    transportMotor.set((transportSpeedEntry.getDouble(0.0))*(multiplier)*(0.8));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
