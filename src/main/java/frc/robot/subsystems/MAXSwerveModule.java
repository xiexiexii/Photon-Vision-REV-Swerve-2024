// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

// MAX Swerve Module Class
public class MAXSwerveModule {
    
    // Motors, Encoders, PID for the driving and turning motors <3
    private final CANSparkMax m_drivingSparkMax;
    private final CANSparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkPIDController m_drivingPIDController;
    private final SparkPIDController m_turningPIDController;

    // Ang Offset is the rotation from default position
    private double m_chassisAngularOffset = 0;

    // SMS is the speed and rotation of the Swerve Module
    private SwerveModuleState m_desiredSwerveModuleState = new SwerveModuleState(0.0, new Rotation2d());

    // Constructor for a new MAX Swerve Module yay
    public MAXSwerveModule(int drivingCANID, int turningCANID, double chassisAngularOffset) {

        // Initializes Driving and Turning Motors
        m_drivingSparkMax = new CANSparkMax(drivingCANID, MotorType.kBrushless);
        m_turningSparkMax = new CANSparkMax(turningCANID, MotorType.kBrushless);

        // Factory resets the driving and turning Spark MAX
        // This way, we don't get weird config issues
        m_drivingSparkMax.restoreFactoryDefaults();
        m_turningSparkMax.restoreFactoryDefaults();
        
        // Set up Encoders and PIDS for drive and turn Spark MAX
        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        m_drivingPIDController = m_drivingSparkMax.getPIDController();
        m_turningPIDController = m_turningSparkMax.getPIDController();
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);

        // Converts rotations and RPM to meters and meters/second
        // This is because the encoder and WPILib have different units
        m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        // Converts rotations and RPM to radians and radians/second
        // This time, it's for the turning encoder
        m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        // Invert the turning encoder since the output shaft
        // rotates in the opposite direction of the turning motor
        m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

        // Enables PID wrapping for the turning motor. This allows it
        // to turn through zero to get to the desired position
        m_turningPIDController.setPositionPIDWrappingEnabled(true);
        m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        // Set PID values for the driving motor. May need to
        // be adjusted via PID tuning
        m_drivingPIDController.setP(ModuleConstants.kDrivingP);
        m_drivingPIDController.setI(ModuleConstants.kDrivingI);
        m_drivingPIDController.setD(ModuleConstants.kDrivingD);
        m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

        // Set PID values for the turning motor. May need to 
        // be adjusted for PID tuning
        m_turningPIDController.setP(ModuleConstants.kTurningP);
        m_turningPIDController.setI(ModuleConstants.kTurningI);
        m_turningPIDController.setD(ModuleConstants.kTurningD);
        m_turningPIDController.setFF(ModuleConstants.kTurningFF);
        m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

        // Sets the idle (brake or coast) and the max current.
        m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        // Saves the SparkMAX configs, keeps settings just in
        // case your they brown out during the match
        m_drivingSparkMax.burnFlash();
        m_turningSparkMax.burnFlash();
        
        // Takes care of angular offset and resets
        // driving encoder to zero 
        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredSwerveModuleState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }

    // Returns the current state of the module
    // Subtracts offset to convert field relative to chassis relative
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    // Returns the current position of the module
    // Also does the chassis relative conversion yay
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_drivingEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    // Sets the desired state for the module with speed and angle
    public void setDesiredState(SwerveModuleState desiredState) {
        // Applies angular offset to make it field relative
        SwerveModuleState correctedDesiredState= new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize reference to avoid turning more than 90 degrees
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(m_turningEncoder.getPosition()));

        // Command the Spark MAX towards their set points
        m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        // Sets the corrected desired state as the new desired state
        m_desiredSwerveModuleState = desiredState;
    }

    // Zeros all encoders on SwerveModule
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}
