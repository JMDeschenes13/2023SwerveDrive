package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;


import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final WPI_TalonFX driveMotor;
    private final WPI_VictorSPX turningMotor;

    private final TalonFXSensorCollection driveEncoder;
    private final AnalogEncoder turningEncoder;

    private final PIDController turningPidController;

    //private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        //absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new WPI_TalonFX(driveMotorId);
        turningMotor = new WPI_VictorSPX(turningMotorId);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        driveEncoder = driveMotor.getSensorCollection();
        turningEncoder = new AnalogEncoder(absoluteEncoderId);
        turningEncoder.setDistancePerRotation((2*Math.PI)/ModuleConstants.kTurningMotorGearRatio);
        turningEncoder.reset();
        turningEncoder.setPositionOffset(absoluteEncoderOffsetRad);
        /* 
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        */

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        turningPidController.setTolerance(ModuleConstants.kPositionTolerance);

        resetEncoders();
    }
    public void periodic(){
       

    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public double getDrivePosition() {
        return driveEncoder.getIntegratedSensorPosition()*ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        // Trying to get the output from the encoder into the range from -Pi to Pi
        double position = turningEncoder.getDistance()%(2*Math.PI);
        if (position < 0){
            position += 2*Math.PI;
            position = position % (2*Math.PI);
        }
        position -= Math.PI;
        return position;
        
    }

    public double getDriveVelocity() {
        return (driveEncoder.getIntegratedSensorVelocity()*ModuleConstants.kDriveEncoderRot2Meter)/10;
    }
    /* 
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }
    */
    /* 
    public double getAbsoluteEncoderRad() {
        double angle = turningEncoder.getAbsolutePosition() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }
    */
    public void resetEncoders() {
        driveEncoder.setIntegratedSensorPosition(0,100);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set((state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond));
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putNumber("TurnMotor"+turningEncoder.getChannel()+"Speed", turningMotor.get());
        SmartDashboard.putNumber("Set Motor" + turningEncoder.getChannel() + "to", turningPidController.calculate(getTurningPosition(), state.angle.getRadians()) );
        SmartDashboard.putNumber("Swerve[" + turningEncoder.getChannel() + "] state", state.angle.getRadians());
    }

    public void stop() { 
        driveMotor.set(0);
        turningMotor.set(0);
    }
}

