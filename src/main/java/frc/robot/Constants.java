// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Swerve {
    public static final double maxSpeed = 4.0;
    public static final boolean invertGyro = false;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.85714286; // old value 1 / 5.8462
    public static final double kTurningMotorGearRatio = 1 / 12.8; // old value 1 / 18.0
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    /* Drivetrain Constants IN METERS */
    public static final double trackWidth = Units.inchesToMeters(21); // TODO: This must be tuned to specific robot
    public static final double wheelBase = Units.inchesToMeters(24);
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int DRIVE_ID = 19;
      public static final int STEER_ID = 10;
      public static final int STEER_ENCODER_ID = 12;
      public static final Rotation2d OFFSET = Rotation2d.fromRadians((-4.85 * Math.PI) / 4);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int DRIVE_ID = 20;
      public static final int STEER_ID = 2;
      public static final int STEER_ENCODER_ID = 15;
      public static final Rotation2d OFFSET = Rotation2d.fromRadians(1.13 * Math.PI);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int DRIVE_ID = 11;
      public static final int STEER_ID = 6;
      public static final int STEER_ENCODER_ID = 9;
      public static final Rotation2d OFFSET = Rotation2d.fromRadians((-3.45 * Math.PI) / 4);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int DRIVE_ID = 14;
      public static final int STEER_ID = 5;
      public static final int STEER_ENCODER_ID = 10;
      public static final Rotation2d OFFSET = Rotation2d.fromRadians((0.75 * Math.PI) / 2);
    }
  }
}
