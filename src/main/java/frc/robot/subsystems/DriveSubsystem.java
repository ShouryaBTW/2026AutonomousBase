package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Variables;
import frc.robot.utils.APOdometry;
import frc.robot.utils.Calculations;
import frc.robot.utils.LimelightToAPOTranslator;
import frc.robot.utils.Pose;
import frc.robot.utils.Vector;

public class DriveSubsystem extends SubsystemBase {

  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGryoID);

  private final APOdometry m_odometry;
  private final Field2d m_field = new Field2d();
  private Pose2d m_simPose = new Pose2d();
  private ChassisSpeeds m_simChassisSpeeds = new ChassisSpeeds();
  private double m_lastSimTime = Timer.getFPGATimestamp();

  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    List<MAXSwerveModule> modules = List.of(
        m_frontLeft, m_frontRight, m_rearLeft, m_rearRight);

    m_odometry = APOdometry.getInstance(modules, m_gyro);

    // Publish the estimated robot position so WPILib's Simulation GUI and Glass
    // can display it on a 2D field.
    SmartDashboard.putData("Field", m_field);
  }

  @Override
public void periodic() {
  m_odometry.update();
  Pose pose = getPose();
  // The autonomous routines use a field frame with negative X/Y coordinates.
  // Rotate that frame 180 degrees for Field2d, whose field display uses the
  // standard positive-X/positive-Y field coordinates. This changes display only.
  m_field.setRobotPose(
      -pose.getX(),
      -pose.getY(),
      Rotation2d.fromDegrees(pose.getAngle() + 180.0));
}

  /** Advances a simple ideal swerve model for desktop simulation. */
  public void simulationPeriodic() {
    double now = Timer.getFPGATimestamp();
    double dt = Math.min(now - m_lastSimTime, 0.05);
    m_lastSimTime = now;

    if (!DriverStation.isEnabled()) {
      m_simChassisSpeeds = new ChassisSpeeds();
    }

    if (dt > 0.0) {
      m_simPose = m_simPose.exp(
          new Twist2d(
              m_simChassisSpeeds.vxMetersPerSecond * dt,
              m_simChassisSpeeds.vyMetersPerSecond * dt,
              m_simChassisSpeeds.omegaRadiansPerSecond * dt));
    }
  }

  public Pose getPose() {
    if (RobotBase.isSimulation()) {
      return new Pose(
          m_simPose.getX(), m_simPose.getY(),
          Calculations.normalizeAngle360(m_simPose.getRotation().getDegrees()));
    }
    return m_odometry.getPose();
  }

  public Pose getPoseContinuous() {
    if (RobotBase.isSimulation()) {
      return getPose();
    }
    return m_odometry.getPoseContinuous();
  }

  public Vector getFieldVelocity() {
    Vector velocity = m_odometry.getVelocity();
    return new Vector(velocity.getX(), velocity.getY());
  }

  public void setPose(Pose pose) {
    m_odometry.setPose(pose);
    if (RobotBase.isSimulation()) {
      m_simPose = new Pose2d(
          pose.getX(), pose.getY(), Rotation2d.fromDegrees(pose.getAngle()));
    }
  }

  public void resetToOrigin() {
    m_odometry.reset();
    if (RobotBase.isSimulation()) {
      m_simPose = new Pose2d();
    }
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    if (RobotBase.isSimulation()) {
      m_simChassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(swerveModuleStates);
    }

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    if (RobotBase.isSimulation()) {
      m_simChassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(desiredStates);
    }

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Base robot heading used everywhere outside APOdometry.
   * Keep this convention aligned with your real robot tests.
   */
  public double getHeading() {
    if (RobotBase.isSimulation()) {
      return Calculations.normalizeAngle360(m_simPose.getRotation().getDegrees());
    }
    double heading = m_gyro.getRotation2d().getDegrees()
        * (DriveConstants.kGyroReversed ? -1.0 : 1.0);

    return Calculations.normalizeAngle360(heading);
  }

  public double getTurnRate() {
    if (RobotBase.isSimulation()) {
      return Math.toDegrees(m_simChassisSpeeds.omegaRadiansPerSecond);
    }
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble()
        * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setGyroYaw(double yawDeg) {
    double rawYaw = DriveConstants.kGyroReversed ? -yawDeg : yawDeg;
    m_gyro.setYaw(rawYaw);
    if (RobotBase.isSimulation()) {
      m_simPose = new Pose2d(m_simPose.getTranslation(), Rotation2d.fromDegrees(yawDeg));
    }
  }

  public void resetGyro() {
    setGyroYaw(0.0);
  }

  public double getVisionYaw() {
  return getHeading();
}

public double getVisionYawRate() {
  return getTurnRate();
}

  /**
   * Hardware init only.
   * Do not force a field heading here.
   */
  public void initializeGyro() {
    Timer timer = new Timer();
    timer.start();

    while (Double.isNaN(m_gyro.getRotation2d().getDegrees()) && timer.get() < 2.0) {
      Timer.delay(0.01); 
    }
  }

  /**
 * Sets odometry pose from Limelight, but rejects large jumps.
 */
/**
 * Applies Limelight position as a translation correction to odometry.
 * Keeps the current drivetrain heading and wheel state intact.
 *
 * This does NOT hard-reset odometry like setPose().
 */
public void correctPositionFromLimelight() {
  if (!Variables.limelight.hasTarget) {
    return;
  }

  Pose visionPose = LimelightToAPOTranslator.getTranslatedPose();
  Pose currentPose = getPose();

  double dx = visionPose.getX() - currentPose.getX();
  double dy = visionPose.getY() - currentPose.getY();

  m_odometry.translatePose(dx, dy);
}
}
