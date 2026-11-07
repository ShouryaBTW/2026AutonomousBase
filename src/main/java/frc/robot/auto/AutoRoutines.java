package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.DriveSubsystem;

/** Builds autonomous routes using point-to-point drivetrain commands only. */
public class AutoRoutines {
  private final DriveSubsystem robotDrive;

  public AutoRoutines(DriveSubsystem robotDrive) {
    this.robotDrive = robotDrive;
  }

  public Command getCommandForMode(AutoMode mode) {
    return switch (mode) {
      case RED_RIGHT -> buildRedRightAuto();
      case RED_LEFT -> buildRedLeftAuto();
      case BLUE_RIGHT -> buildBlueRightAuto();
      case BLUE_LEFT -> buildBlueLeftAuto();
    };
  }

  private Command buildRedRightAuto() {
    return new SequentialCommandGroup(
        point(9.4, 7.3, 270, 0.25, 1, 1.35),
        point(8.4, 4.5, 270, 0.1, 0.8, 0.65),
        point(10, 5.625, 315, 0.15, 0.8, 0.8),
        point(13.5, 5.625, 315, 0.25, 0.8, 0.65),
        point(13.5, 7.3, 270, 0.1, 0.8, 0.65),
        point(9.4, 7.3, 270, 0.25, 1, 1.35),
        point(9.4, 4.5, 270, 0.1, 0.8, 0.85),
        point(9.4, 5.625, 315, 0.5, 0.8, 0.8),
        point(13.5, 5.625, 315, 0.25, 0.8, 0.65))
        .finallyDo(interrupted -> robotDrive.drive(0.0, 0.0, 0.0, true));
  }

  private Command buildRedLeftAuto() {
    return new SequentialCommandGroup(
        point(9.4, 0.6, 90, 0.25, 1, 1.35),
        point(8.4, 4.5, 90, 0.1, 0.8, 0.65),
        point(10, 2.5, 35, 0.15, 0.8, 0.8),
        point(13.5, 2.5, 35, 0.25, 0.8, 0.65),
        point(13.5, 0.6, 90, 0.1, 0.8, 0.65),
        point(9.4, 0.6, 90, 0.25, 1, 1.35),
        point(9.4, 4, 90, 0.1, 0.8, 0.85),
        point(9.4, 2.5, 45, 0.5, 0.8, 0.8),
        point(13.5, 2.5, 45, 0.25, 0.8, 0.65))
        .finallyDo(interrupted -> robotDrive.drive(0.0, 0.0, 0.0, true));
  }

  private Command buildBlueRightAuto() {
    return new SequentialCommandGroup(
        point(7.4, 0.65, 90, 0.25, 1, 1.35),
        point(8.4, 4.5, 90, 0.1, 0.8, 0.65),
        point(6.8, 2.5, 135, 0.15, 0.8, 0.8),
        point(3.1, 2.5, 135, 0.25, 0.8, 0.65),
        point(3.1, 0.65, 90, 0.1, 0.8, 0.65),
        point(7.4, 0.65, 90, 0.25, 1, 1.35),
        point(7.4, 4.5, 90, 0.1, 0.8, 0.85),
        point(7.4, 2.5, 135, 0.5, 0.8, 0.8),
        point(3.1, 2.5, 135, 0.25, 0.8, 0.65))
        .finallyDo(interrupted -> robotDrive.drive(0.0, 0.0, 0.0, true));
  }

  private Command buildBlueLeftAuto() {
    return new SequentialCommandGroup(
        point(7.4, 7.3, 270, 0.25, 1, 1.35),
        point(8.4, 4.5, 270, 0.1, 0.8, 0.65),
        point(6.8, 5.625, 215, 0.1, 0.8, 0.8),
        point(3.1, 5.625, 215, 0.1, 0.8, 0.65),
        point(3.1, 7.4, 270, 0.1, 0.8, 0.65),
        point(7.4, 7.3, 270, 0.1, 1, 1.35),
        point(7.4, 4.5, 270, 0.1, 0.8, 0.85),
        point(7.4, 5.625, 215, 0.25, 0.8, 0.65),
        point(3.1, 5.625, 215, 0.25, 0.8, 0.65))
        .finallyDo(interrupted -> robotDrive.drive(0.0, 0.0, 0.0, true));
  }

  private DriveToPoint point(
      double x, double y, double heading, double tolerance, double p, double maxSpeed) {
    return new DriveToPoint(
        robotDrive,
        x,
        y,
        heading,
        tolerance,
        0.02,
        true,
        p,
        maxSpeed,
        p,
        maxSpeed);
  }
}
