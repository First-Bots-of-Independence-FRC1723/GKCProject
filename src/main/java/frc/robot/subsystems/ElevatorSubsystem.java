// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSub. */
  
  private VictorSPX spoolMotorLeft = new VictorSPX(Constants.Elevator.spoolMotorLeftPort); // robot's left (intake motor)
  private VictorSPX spoolMotorRight = new VictorSPX(Constants.Elevator.spoolMotorRightPort);
  private VictorSPX angleMotorLeft = new VictorSPX(Constants.Elevator.angleMotorLeftPort);
  private VictorSPX angleMotorRight = new VictorSPX(Constants.Elevator.angleMotorRightPort);

  private ShuffleboardTab elevatorTab = Shuffleboard.getTab("elevator");
  private NetworkTableEntry elevatorSpeedEntry = elevatorTab.add("elevator speed", 0.8)
  .withWidget(BuiltInWidgets.kNumberSlider)
  .withProperties(Map.of("min", 0, "max", 1))
  .getEntry();

  private NetworkTableEntry elevatorAngleSpeedEntry = elevatorTab.add("elevator angle speed", 0.3)
  .withWidget(BuiltInWidgets.kNumberSlider)
  .withProperties(Map.of("min", 0, "max", 1))
  .getEntry();

  public ElevatorSubsystem() {
    spoolMotorLeft.setNeutralMode(NeutralMode.Brake);
    spoolMotorRight.setNeutralMode(NeutralMode.Brake);
    angleMotorLeft.setNeutralMode(NeutralMode.Brake);
    angleMotorRight.setNeutralMode(NeutralMode.Brake);
  }

  public void spool(double multiplier){
    spoolMotorLeft.set(VictorSPXControlMode.PercentOutput, (elevatorSpeedEntry.getDouble(0.0))*(multiplier)*(0.8));
    spoolMotorRight.set(VictorSPXControlMode.PercentOutput, (elevatorSpeedEntry.getDouble(0.0))*(multiplier)*(-0.8));
  }

  public void spoolRight(double multiplier){
    spoolMotorRight.set(VictorSPXControlMode.PercentOutput, (0.6)*(multiplier)*(0.8));
  }

  public void spoolLeft(double multiplier){
    spoolMotorLeft.set(VictorSPXControlMode.PercentOutput, (0.6)*(multiplier)*(-0.8));
  }
  
  public void togglePistons(){
    /* if(elevatorSolenoid.get() == Value.kForward){
      elevatorSolenoid.set(Value.kReverse);
    }
    else{
      elevatorSolenoid.set(Value.kForward);
    }
    */
  }

  public void closeGrabbyGrabby(){
    // elevatorSolenoid.set(Value.kForward);
  }

  public void openGrabbyGrabby(){
    /// elevatorSolenoid.set(Value.kReverse);
  }

  // add back limit switches
  /*
  public void angleMotor(double multiplier){
    if(multiplier > 0){
      if(!frontLeftLimitSwitch.get()){
        angleMotorLeft.set(VictorSPXControlMode.PercentOutput, (elevatorAngleSpeedEntry.getDouble(0.0))*(multiplier)*(0.8));
      }
      if(!frontRightLimitSwitch.get()){
        angleMotorRight.set(VictorSPXControlMode.PercentOutput, (elevatorAngleSpeedEntry.getDouble(0.0))*(multiplier)*(-0.8));
      }
    }else if(multiplier < 0){
      if(!backRightLimitSwitch.get()){
        angleMotorLeft.set(VictorSPXControlMode.PercentOutput, (elevatorAngleSpeedEntry.getDouble(0.0))*(multiplier)*(0.8));
        angleMotorRight.set(VictorSPXControlMode.PercentOutput, (elevatorAngleSpeedEntry.getDouble(0.0))*(multiplier)*(-0.8));
      }
    }
  }
 */

  public void angleMotorSimple(double multiplier){
    angleMotorLeft.set(VictorSPXControlMode.PercentOutput, (elevatorAngleSpeedEntry.getDouble(0.0))*(multiplier)*(0.8));
    angleMotorRight.set(VictorSPXControlMode.PercentOutput, (elevatorAngleSpeedEntry.getDouble(0.0))*(multiplier)*(-0.8));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

   /*
    frontLeftLimitSwitchEntry.setBoolean(frontLeftLimitSwitch.get());
    frontRightLimitSwitchEntry.setBoolean(frontRightLimitSwitch.get());
    backRightLimitSwitchEntry.setBoolean(backRightLimitSwitch.get());
    */
  }
}
