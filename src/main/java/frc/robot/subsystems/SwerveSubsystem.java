package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    //private SwerveModuleState[] swerveModuleStates = {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};   
    private SwerveModulePosition[] swerveModulePositions = {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),backRight.getPosition()};
    private final PigeonIMU gyro = new PigeonIMU(32);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), swerveModulePositions);
    

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.setYaw(0);
    }

    public double getHeading() {
        return (gyro.getYaw());
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), swerveModulePositions, pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), swerveModulePositions);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().toString());
        SmartDashboard.putNumber("FrontLeftDrivePosition", frontLeft.getDrivePosition());
        SmartDashboard.putNumber("FrontLeftSpeed", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("FrontLeftTurnPosition", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("FrontRightDrivePosition", frontRight.getDrivePosition());
        SmartDashboard.putNumber("FrontRightSpeed", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("FrontRightTurnPosition", frontRight.getTurningPosition());
        SmartDashboard.putNumber("BackLeftDrivePosition", backLeft.getDrivePosition());
        SmartDashboard.putNumber("BackLeftSpeed", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("backLeftTurnPosition", backLeft.getTurningPosition());
        SmartDashboard.putNumber("backRightDrivePostion", backRight.getDrivePosition());
        SmartDashboard.putNumber("backRightSpeed", backRight.getDriveVelocity());
        SmartDashboard.putNumber("backRightTurnPosition", backRight.getTurningPosition());
        

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SmartDashboard.putString("State 0", desiredStates[0].toString());
        SmartDashboard.putString("State 1", desiredStates[1].toString());
        SmartDashboard.putString("State 2", desiredStates[2].toString());
        SmartDashboard.putString("State 3", desiredStates[3].toString());
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
