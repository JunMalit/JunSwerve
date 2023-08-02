package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;

public class DrivetrainSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;

    public frc.lib.util.SwerveModule[] m_SwerveMods;

    private AHRS m_navX = new AHRS(SPI.Port.kMXP, (byte) 0);

    private SwerveDriveOdometry m_odometry;

    public DrivetrainSubsystem() {

        m_SwerveMods = new SwerveModule[] {
                new SwerveModule(Mod0.DRIVE_ID, Mod0.STEER_ENCODER_ID, Mod0.STEER_ID, 0),
                new SwerveModule(Mod1.DRIVE_ID, Mod1.STEER_ENCODER_ID, Mod1.STEER_ID, 1),
                new SwerveModule(Mod2.DRIVE_ID, Mod2.STEER_ENCODER_ID, Mod2.STEER_ID, 2),
                new SwerveModule(Mod3.DRIVE_ID, Mod3.STEER_ENCODER_ID, Mod3.STEER_ID, 3)
        };
        for (SwerveModule mod : m_SwerveMods) {
            mod.reset();
        }

        m_odometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, null, getModulePositions());
    }

    /**
     * 
     * @param translation     X and Y values (multiply by max speed)
     * @param rotation        Theta value (multiply by max angular velocity)
     * @param currentRotation The current rotation/theta of the robot (i.e. the yaw
     *                        from the gyro)
     * @param fieldRelative   Is field relative
     * @param isOpenLoop
     */
    public void drive(Translation2d translation, double rotation, Rotation2d currentRotation, boolean fieldRelative,
            boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        currentRotation)
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

        setModuleStates(swerveModuleStates);
    }

    public double getYaw() {
        return (Constants.Swerve.invertGyro) ? (360 - m_navX.getYaw())
                : (m_navX.getYaw());
    }

    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }

    // public void zeroGyro(){
    // m_navX.reset();
    // }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                chassisSpeeds);
        setModuleStates(swerveModuleStates);
    }

    /**
     * Overloaded drive method.
     * 
     * @param chassisSpeeds       to drive at.
     * @param speedCapTranslation in m/s.
     * @param speedCapRotation    in rad/s.
     */
    public void drive(ChassisSpeeds chassisSpeeds, double speedCapTranslation, double speedCapRotation) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, chassisSpeeds, speedCapTranslation,
                speedCapTranslation, speedCapRotation);
        setModuleStates(swerveModuleStates);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : m_SwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : m_SwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_SwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    @Override
    public void periodic() {
        // for (SwerveModule mod : m_SwerveMods) {
        // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Position",
        // mod.getDrivePosition());
        // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Turn Position",
        // mod.getTurningPosition());
        // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
        // mod.getState().speedMetersPerSecond);
        // }
        m_odometry.update(Rotation2d.fromDegrees(getYaw()), getModulePositions());
    }
}