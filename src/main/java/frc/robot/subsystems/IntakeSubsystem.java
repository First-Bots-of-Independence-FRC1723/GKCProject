// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private CANSparkMax intakeMotor = new CANSparkMax(Constants.Balls.intakeMotorPort, MotorType.kBrushless);

  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);

  private ShuffleboardTab intakeTab = Shuffleboard.getTab("intake");
  private NetworkTableEntry intakeSpeedEntry = intakeTab.add("intake speed", 0.2).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
  
  public IntakeSubsystem() {}

  public void intake(double multiplier){
    //if(intakeSolenoid.get() == Value.kForward){
      intakeMotor.set((intakeSpeedEntry.getDouble(0.0))*(multiplier)*(0.8));
    //}
  }

  
  public void togglePistons(){
    if(intakeSolenoid.get() == Value.kForward){
      intakeSolenoid.set(Value.kReverse);
    }
    else{
      intakeSolenoid.set(Value.kForward);
    }
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
