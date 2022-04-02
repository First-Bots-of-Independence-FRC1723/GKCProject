// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private VictorSPX hoodMotor = new VictorSPX(Constants.Hood.hoodMotorPort);

  public double tX;
  public double tY;
  public double tVert;

  private ShuffleboardTab hoodTab = Shuffleboard.getTab("hood tab");

  private NetworkTableEntry hoodSpeedEntry = hoodTab.add("hood speed", 0.6).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

  // private NetworkTableEntry kPEntry = hoodTab.add("P", 0.9).getEntry();

  public HoodSubsystem() {}

  public void angleHood(double multiplier){
    hoodMotor.set(ControlMode.PercentOutput, (hoodSpeedEntry.getDouble(0.0))*(multiplier)*(0.8));
  }

  
  public void angleHoodToCount(int setpoint){
    // double percentError = (setpoint - potentiometer.getValue())/Constants.Hood.tenthRing; // should be max value, tenth ring needs to be farthest we can possibly go
    // hoodMotor.set(ControlMode.PercentOutput, (percentError*kPEntry.getDouble(0.0))*0.8);
  }

  public void aimHoodWithLimelight(){
    // ranges that the hood angle is set to based on distance
    if(tVert>=0){
      angleHoodToCount(Constants.Hood.firstRing);
    }
    else if(tVert>=0){
      angleHoodToCount(Constants.Hood.secondRing);
    }
    else if(tVert>=0){
      angleHoodToCount(Constants.Hood.thirdRing);
    }
    else if(tVert>=0){
      angleHoodToCount(Constants.Hood.fourthRing);
    }
    else if(tVert>=0){
      angleHoodToCount(Constants.Hood.fifthRing);
    }
    else if(tVert>=0){
      angleHoodToCount(Constants.Hood.sixthRing);
    }
    else if(tVert>=0){
      angleHoodToCount(Constants.Hood.seventhRing);
    }
    else if(tVert>=0){
      angleHoodToCount(Constants.Hood.eighthRing);
    }
    else if(tVert>=0){
      angleHoodToCount(Constants.Hood.ninthRing);
    }
    else if(tVert>=0){
      angleHoodToCount(Constants.Hood.tenthRing);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
