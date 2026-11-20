package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Variables;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DataLog extends SubsystemBase {
  private final Supplier<String> selectedTurnTargetNameSupplier;
  private final DoubleSupplier selectedTurnTargetXSupplier;
  private final DoubleSupplier selectedTurnTargetYSupplier;
  private final Supplier<Pose> robotPoseSupplier;
  private final Field2d field = new Field2d();

  public DataLog(
      Supplier<String> selectedTurnTargetNameSupplier,
      DoubleSupplier selectedTurnTargetXSupplier,
      DoubleSupplier selectedTurnTargetYSupplier,
      Supplier<Pose> robotPoseSupplier) {
    this.selectedTurnTargetNameSupplier = selectedTurnTargetNameSupplier;
    this.selectedTurnTargetXSupplier = selectedTurnTargetXSupplier;
    this.selectedTurnTargetYSupplier = selectedTurnTargetYSupplier;
    this.robotPoseSupplier = robotPoseSupplier;

    SmartDashboard.putData("Field", field);
  }

  @Override
public void periodic() {
    Pose robotPose = robotPoseSupplier.get();
    field.setRobotPose(
        new Pose2d(
            robotPose.getX(),
            robotPose.getY(),
            Rotation2d.fromDegrees(robotPose.getAngle())));

    SmartDashboard.putBoolean("LL/HasTarget", Variables.limelight.hasTarget);
    SmartDashboard.putNumber("LL/TID", Variables.limelight.tid);
    SmartDashboard.putNumber("LL/TagCount", Variables.limelight.tagCount);
    SmartDashboard.putNumber("LL/LatencyMs", Variables.limelight.latencyMs);

    SmartDashboard.putNumber("LL/X", Variables.limelight.ll_x);
    SmartDashboard.putNumber("LL/Y", Variables.limelight.ll_y);
    SmartDashboard.putNumber("LL/Rot", Variables.limelight.ll_rot);

    SmartDashboard.putNumber("Drive/X", Variables.drive.currentX);
    SmartDashboard.putNumber("Drive/Y", Variables.drive.currentY);
    SmartDashboard.putNumber("Drive/Heading", Variables.drive.heading);
    SmartDashboard.putNumber("Drive/TurnRate", Variables.drive.turnRate);
    SmartDashboard.putNumber(
        "Heading/LLMinusDriveWrappedDeg",
        Calculations.shortestAngularDistance(Variables.limelight.ll_rot, Variables.drive.heading));

    SmartDashboard.putNumber("Error/X", Variables.limelight.ll_x - Variables.drive.currentX);
    SmartDashboard.putNumber("Error/Y", Variables.limelight.ll_y - Variables.drive.currentY);
    SmartDashboard.putNumber("Error/Rot", Variables.limelight.ll_rot - Variables.drive.heading);

    SmartDashboard.putNumber("TurnTarget/Red/X", Constants.TurnTargetConstants.kRedHubX);
    SmartDashboard.putNumber("TurnTarget/Red/Y", Constants.TurnTargetConstants.kRedHubY);
    SmartDashboard.putNumber("TurnTarget/Blue/X", Constants.TurnTargetConstants.kBlueHubX);
    SmartDashboard.putNumber("TurnTarget/Blue/Y", Constants.TurnTargetConstants.kBlueHubY);

    SmartDashboard.putString("TurnTarget/SelectedName", selectedTurnTargetNameSupplier.get());
    SmartDashboard.putNumber("TurnTarget/SelectedX", selectedTurnTargetXSupplier.getAsDouble());
    SmartDashboard.putNumber("TurnTarget/SelectedY", selectedTurnTargetYSupplier.getAsDouble());
    SmartDashboard.putNumber("TurnTarget/TargetAngleDeg", Variables.drive.targetHubAngleDeg);
    SmartDashboard.putNumber("TurnTarget/AngleErrorDeg", Variables.drive.targetHubAngleErrorDeg);

    SmartDashboard.putNumber("Shooter RPS", Variables.shooter.shooterRPS);
    SmartDashboard.putNumber("Pivot Position", Variables.pivot.pivotPosition);
    SmartDashboard.putNumber("Distance From Hub", Variables.distanceMeters);
  }
}
