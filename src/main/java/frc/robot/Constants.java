package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Constants required by the drivetrain and swerve modules. */
public final class Constants {
  private Constants() {}

  public static final class DriveConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverDeadband = 0.08;
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kTrackWidth = Units.inchesToMeters(27.5);
    public static final double kWheelBase = Units.inchesToMeters(27);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 12;
    public static final int kFrontRightDrivingCanId = 13;
    public static final int kRearRightDrivingCanId = 14;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 4;

    public static final int kGryoID = 24;
    public static final boolean kGyroReversed = false;

    private DriveConstants() {}
  }

  public static final class ModuleConstants {
    public static final int kDrivingMotorPinionTeeth = 14;
    public static final double kDrivingMotorFreeSpeedRps = KrakenMotorConstants.kFreeSpeedRps;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        kDrivingMotorFreeSpeedRps / kDrivingMotorReduction;

    private ModuleConstants() {}
  }

  public static final class KrakenMotorConstants {
    public static final double kFreeSpeedRps = 100;

    private KrakenMotorConstants() {}
  }

}
