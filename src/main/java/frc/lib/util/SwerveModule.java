// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

/**
 * Code for a Swerve Module using NEOs
 * Necessary constants in your Constants.Swerve class:
 * double maxSpeed
 */
public class SwerveModule {
    // Declare our two motors
    // One swerve module runs off two motors, a drive motor and a steer motor
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_steerMotor;

    // Declare our encoders
    // We use a CANCoder for the steer motor because we need our angle measurements
    // to be absolute
    // Since the measurement is absolute, the motor is always aware of its position
    // regardless of it being turned off and moved
    private RelativeEncoder m_driveEncoder;
    private CANCoder m_steerEncoder;

    // Declare PID Controller
    private ProfiledPIDController m_steerPIDController;

    // Allows us to remember the module's angle from the last run instance.
    // We set the module to this angle if the requested drive speed is negligible
    // This prevents unwanted jittering when the joystick is not being touched.
    private Rotation2d lastAngle;

    /**
     * Used to ID each swerve module.
     * FL - 0
     * FR - 1
     * BL - 2
     * BR - 3
     */
    public int moduleNumber;

    /**
     * Creates a new Swerve Module.
     * 
     * @param driveID        ID for the drive motor
     * @param steerID        ID for the steer motor
     * @param steerEncoderID ID for the module CANCoder
     */
    public SwerveModule(int driveID, int steerID, int steerEncoderID, int moduleNumber) {

        // Instantiate both motors according to given IDs
        m_driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        m_driveMotor = new CANSparkMax(steerID, MotorType.kBrushless);

        // Instantiate both encoders
        m_driveEncoder = m_driveMotor.getEncoder();
        m_steerEncoder = new CANCoder(steerEncoderID);

        // Set up steering PID controller so the module can snap to its desired angle.
        // This PID controller is responsible for calculating the motor speeds required
        // to match the module's current angle to the desired angle.
        m_steerPIDController = new ProfiledPIDController(0.4, 0, 0,
                new TrapezoidProfile.Constraints(20.0 * 2.0 * Math.PI, 20.0 * 2.0 * Math.PI));

        lastAngle = Rotation2d.fromRadians(m_steerEncoder.getAbsolutePosition());

        // Tell PID controller that it is a *wheel*
        m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_driveEncoder.setPosition(0);

        configAngleMotor();
        configDriveMotor();
    }

    // Methods to get module information from both motors.

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(),
                Rotation2d.fromRadians(m_steerEncoder.getAbsolutePosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(),
                Rotation2d.fromRadians(m_steerEncoder.getAbsolutePosition()));
    }

    // Methods to move the motors.

    /**
     * 
     * @param desiredState
     */
    private void setAngle(SwerveModuleState desiredState) {

        // Changes module angle.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.001))
                ? lastAngle
                : desiredState.angle; // Prevent rotating module if drive speed is less then 1%. Prevents Jittering.

        // Uses the PID controller to calculate steer speeds necessary to reach the
        // desired module angle
        m_steerMotor.set(m_steerPIDController.calculate(m_steerEncoder.getAbsolutePosition(),
                desiredState.angle.getRadians()));

        lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState) {
        double percentOutput = (Math.abs(desiredState.speedMetersPerSecond) <= Constants.Swerve.maxSpeed * 0.01) ? 0
                : desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
        m_driveMotor.set(percentOutput);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState,
                Rotation2d.fromRadians(m_steerEncoder.getAbsolutePosition()));
        setAngle(desiredState);
        setSpeed(desiredState);
    }

    public void rawSet(double drive, double turn) {
        m_driveMotor.set(drive);
        m_steerMotor.set(turn);
    }

    public void stop() {
        m_steerMotor.stopMotor();
        m_driveMotor.stopMotor();
    }

    private void configAngleMotor() {
        m_steerMotor.setInverted(false);
    }

    private void configDriveMotor() {
        m_driveMotor.setInverted(false);
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_driveMotor.burnFlash();
        m_driveEncoder.setPositionConversionFactor(Constants.Swerve.kDriveEncoderRot2Meter);
        m_driveEncoder.setVelocityConversionFactor(Constants.Swerve.kDriveEncoderRPM2MeterPerSec);

        // Change conversion factors for CANCoder - should be in radians!
        m_steerEncoder.configFeedbackCoefficient(Constants.Swerve.kTurningEncoderRot2Rad, "rad",
                SensorTimeBase.PerSecond);
    }

    public void reset() {
        m_driveEncoder.setPosition(0);
        m_steerEncoder.setPosition(0);
        m_steerPIDController.reset(0);
    }
}
