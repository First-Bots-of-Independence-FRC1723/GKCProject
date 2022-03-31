package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AnalogGyro gyro;

    public static ShuffleboardTab swerveTab = Shuffleboard.getTab("swerve");
    private static NetworkTableEntry mod0Can;
    private static NetworkTableEntry mod1Can;
    private static NetworkTableEntry mod2Can;
    private static NetworkTableEntry mod3Can;
    private static NetworkTableEntry mod0Int;
    private static NetworkTableEntry mod1Int;
    private static NetworkTableEntry mod2Int;
    private static NetworkTableEntry mod3Int;
    private static NetworkTableEntry mod0Vel;
    private static NetworkTableEntry mod1Vel;
    private static NetworkTableEntry mod2Vel;
    private static NetworkTableEntry mod3Vel;
    private static NetworkTableEntry mod0Drive;
    private static NetworkTableEntry mod1Drive;
    private static NetworkTableEntry mod2Drive;
    private static NetworkTableEntry mod3Drive;

    private static NetworkTableEntry gyroAngle;

    public DriveSubsystem() {
        gyro = new AnalogGyro(0);
        gyro.initGyro();
        gyro.calibrate();
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        setUpShuffleboard();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getAngle()) : Rotation2d.fromDegrees(gyro.getAngle());
    }

    private void setUpShuffleboard(){
        mod0Can = swerveTab.add("Mod 0 Cancoder Angle", mSwerveMods[0].getCanCoder().getDegrees()).withSize(2, 1).withPosition(5, 1).getEntry();
        mod1Can = swerveTab.add("Mod 1 Cancoder Angle", mSwerveMods[1].getCanCoder().getDegrees()).withSize(2, 1).withPosition(7, 1).getEntry();
        mod2Can = swerveTab.add("Mod 2 Cancoder Angle", mSwerveMods[2].getCanCoder().getDegrees()).withSize(2, 1).withPosition(9, 1).getEntry();
        mod3Can = swerveTab.add("Mod 3 Cancoder Angle", mSwerveMods[3].getCanCoder().getDegrees()).withSize(2, 1).withPosition(11, 1).getEntry();
    
        mod0Int = swerveTab.add("Mod 0 Integrated Angle", mSwerveMods[0].getState().angle.getDegrees()).withSize(2, 1).withPosition(5, 2).getEntry();
        mod1Int = swerveTab.add("Mod 1 Integrated Angle", mSwerveMods[1].getState().angle.getDegrees()).withSize(2, 1).withPosition(7, 2).getEntry();
        mod2Int = swerveTab.add("Mod 2 Integrated Angle", mSwerveMods[2].getState().angle.getDegrees()).withSize(2, 1).withPosition(9, 2).getEntry();
        mod3Int = swerveTab.add("Mod 3 Integrated Angle", mSwerveMods[3].getState().angle.getDegrees()).withSize(2, 1).withPosition(11, 2).getEntry();
    
        mod0Vel = swerveTab.add("Mod 0 Velocity", mSwerveMods[0].getState().speedMetersPerSecond).withSize(2, 1).withPosition(5, 3).getEntry();
        mod1Vel = swerveTab.add("Mod 1 Velocity", mSwerveMods[1].getState().speedMetersPerSecond).withSize(2, 1).withPosition(7, 3).getEntry();
        mod2Vel = swerveTab.add("Mod 2 Velocity", mSwerveMods[2].getState().speedMetersPerSecond).withSize(2, 1).withPosition(9, 3).getEntry();
        mod3Vel = swerveTab.add("Mod 3 Velocity", mSwerveMods[3].getState().speedMetersPerSecond).withSize(2, 1).withPosition(11, 3).getEntry();

        mod0Drive = swerveTab.add("Mod 0 Drive", mSwerveMods[0].mDriveMotor.getSelectedSensorPosition()).withSize(2, 1).withPosition(5, 4).getEntry();
        mod1Drive = swerveTab.add("Mod 1 Drive", mSwerveMods[1].mDriveMotor.getSelectedSensorPosition()).withSize(2, 1).withPosition(7, 4).getEntry();
        mod2Drive = swerveTab.add("Mod 2 Drive", mSwerveMods[2].mDriveMotor.getSelectedSensorPosition()).withSize(2, 1).withPosition(9, 4).getEntry();
        mod3Drive = swerveTab.add("Mod 3 Drive", mSwerveMods[3].mDriveMotor.getSelectedSensorPosition()).withSize(2, 1).withPosition(11, 4).getEntry();

        gyroAngle = swerveTab.add("gyro angle", getYaw().getDegrees()).withSize(2, 2).withPosition(1, 2).getEntry();
    }

    private void updateShuffleboard(){
        mod0Can.setNumber(mSwerveMods[0].getCanCoder().getDegrees());
        mod1Can.setNumber(mSwerveMods[1].getCanCoder().getDegrees());
        mod2Can.setNumber(mSwerveMods[2].getCanCoder().getDegrees());
        mod3Can.setNumber(mSwerveMods[3].getCanCoder().getDegrees());

        mod0Int.setNumber(mSwerveMods[0].getState().angle.getDegrees());
        mod1Int.setNumber(mSwerveMods[1].getState().angle.getDegrees());
        mod2Int.setNumber(mSwerveMods[2].getState().angle.getDegrees());
        mod3Int.setNumber(mSwerveMods[3].getState().angle.getDegrees());

        mod0Vel.setNumber(mSwerveMods[0].getState().speedMetersPerSecond);
        mod1Vel.setNumber(mSwerveMods[1].getState().speedMetersPerSecond);
        mod2Vel.setNumber(mSwerveMods[2].getState().speedMetersPerSecond);
        mod3Vel.setNumber(mSwerveMods[3].getState().speedMetersPerSecond);

        mod0Drive.setNumber(mSwerveMods[0].mDriveMotor.getSelectedSensorPosition());
        mod1Drive.setNumber(mSwerveMods[1].mDriveMotor.getSelectedSensorPosition());
        mod2Drive.setNumber(mSwerveMods[2].mDriveMotor.getSelectedSensorPosition());
        mod3Drive.setNumber(mSwerveMods[3].mDriveMotor.getSelectedSensorPosition());

        gyroAngle.setNumber(getYaw().getDegrees());
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getStates());
        updateShuffleboard();
    }
}
