// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // beep boop
    public static final class Elevator{
        public static final int frontRightLimitSwitchPort = 0;
        public static final int frontLeftLimitSwitchPort = 1;
        public static final int backRightLimitSwitchPort = 2;
        
        public static final int spoolMotorLeftPort = 17; // robot's left (intake motor side)
        public static final int spoolMotorRightPort = 8;
        public static final int angleMotorLeftPort = 15;
        public static final int angleMotorRightPort = 6;

        public static final int elevatorSolenoidPort = 0;
    }

    public static final class Balls{
        public static final int intakeMotorPort = 21;
        public static final int transportMotorPort = 9;
        public static final int leftShooterMotorPort = 3;
        public static final int rightShooterMotorPort = 20;

        public static final int intakeSolenoidPort = 0;
    }

    public static final class Hood{
        public static final int hoodMotorPort = 7;

        public static final int firstRing = 0;
        public static final int secondRing = 0;
        public static final int thirdRing = 0;
        public static final int fourthRing = 0;
        public static final int fifthRing = 0;
        public static final int sixthRing = 0;
        public static final int seventhRing = 0;
        public static final int eighthRing = 0;
        public static final int ninthRing = 0;
        public static final int tenthRing = 0;
        public static final int eleventhRing = 0;
        public static final int twelfthRing = 0;
        public static final int thirteenthRing = 0;
        public static final int fourteenthRing = 0;
        public static final int fifteenthRing = 0;

        public static final int potentiometerPort = 3;

        public static final double kP = 0.9;
    }

    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.25); 
        public static final double wheelBase = Units.inchesToMeters(25.5);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0);
        public static final double angleGearRatio = (12.8 / 1.0);

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 2.5;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);
        
        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = true;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 19;
            public static final int angleMotorID = 18;
            public static final int canCoderID = 4;
            public static final double angleOffset = 43.4;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        // Front Right Module - Module 1 
        public static final class Mod1 {
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 3;
            public static final double angleOffset = 75;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        // Back Left Module - Module 2 
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 1;
            public static final double angleOffset = 96; 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        // Back Right Module - Module 3 
        public static final class Mod3 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 2;
            public static final double angleOffset = 42;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }      
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }
}
