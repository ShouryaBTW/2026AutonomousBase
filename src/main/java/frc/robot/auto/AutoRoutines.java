package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.DriveSubsystem;

/** Builds autonomous routes as an explicit sequence of drive-to-point commands. */
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
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 6.4, 7.3, 270, 0.25, 0.02, true, 1, 1.35, 1, 1.35)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 8.4, 4.5, 270, 0.1, 0.02, true, 0.8, 0.65, 0.8, 0.65)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 10, 5.625, 315, 0.15, 0.02, true, 0.8, 0.8, 0.8, 0.8)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 13.5, 5.625, 315, 0.25, 0.02, true, 0.8, 0.65, 0.8, 0.65)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 13.5, 7.3, 270, 0.1, 0.02, true, 0.8, 0.65, 0.8, 0.65)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 9.4, 7.3, 270, 0.25, 0.02, true, 1, 1.35, 1, 1.35)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 9.4, 4.5, 270, 0.1, 0.02, true, 0.8, 0.85, 0.8, 0.85)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 9.4, 5.625, 315, 0.5, 0.02, true, 0.8, 0.8, 0.8, 0.8)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 13.5, 5.625, 315, 0.25, 0.02, true, 0.8, 0.65, 0.8, 0.65)));
  }

  private Command buildRedLeftAuto() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 9.4, 0.6, 90, 0.25, 0.02, true, 1, 1.35, 1, 1.35)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 8.4, 4.5, 90, 0.1, 0.02, true, 0.8, 0.65, 0.8, 0.65)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 10, 2.5, 35, 0.15, 0.02, true, 0.8, 0.8, 0.8, 0.8)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 13.5, 2.5, 35, 0.25, 0.02, true, 0.8, 0.65, 0.8, 0.65)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 13.5, 0.6, 90, 0.1, 0.02, true, 0.8, 0.65, 0.8, 0.65)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 9.4, 0.6, 90, 0.25, 0.02, true, 1, 1.35, 1, 1.35)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 9.4, 4, 90, 0.1, 0.02, true, 0.8, 0.85, 0.8, 0.85)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 9.4, 2.5, 45, 0.5, 0.02, true, 0.8, 0.8, 0.8, 0.8)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 13.5, 2.5, 45, 0.25, 0.02, true, 0.8, 0.65, 0.8, 0.65)));
  }

  private Command buildBlueRightAuto() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 7.4, 0.65, 90, 0.25, 0.02, true, 1, 1.35, 1, 1.35)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 8.4, 4.5, 90, 0.1, 0.02, true, 0.8, 0.65, 0.8, 0.65)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 6.8, 2.5, 135, 0.15, 0.02, true, 0.8, 0.8, 0.8, 0.8)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 3.1, 2.5, 135, 0.25, 0.02, true, 0.8, 0.65, 0.8, 0.65)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 3.1, 0.65, 90, 0.1, 0.02, true, 0.8, 0.65, 0.8, 0.65)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 7.4, 0.65, 90, 0.25, 0.02, true, 1, 1.35, 1, 1.35)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 7.4, 4.5, 90, 0.1, 0.02, true, 0.8, 0.85, 0.8, 0.85)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 7.4, 2.5, 135, 0.5, 0.02, true, 0.8, 0.8, 0.8, 0.8)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 3.1, 2.5, 135, 0.25, 0.02, true, 0.8, 0.65, 0.8, 0.65)));
  }

  private Command buildBlueLeftAuto() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 7.4, 7.3, 270, 0.25, 0.02, true, 1, 1.35, 1, 1.35)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 8.4, 4.5, 270, 0.1, 0.02, true, 0.8, 0.65, 0.8, 0.65)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 6.8, 5.625, 215, 0.1, 0.02, true, 0.8, 0.8, 0.8, 0.8)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 3.1, 5.625, 215, 0.1, 0.02, true, 0.8, 0.65, 0.8, 0.65)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 3.1, 7.4, 270, 0.1, 0.02, true, 0.8, 0.65, 0.8, 0.65)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 7.4, 7.3, 270, 0.1, 0.02, true, 1, 1.35, 1, 1.35)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 7.4, 4.5, 270, 0.1, 0.02, true, 0.8, 0.85, 0.8, 0.85)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 7.4, 5.625, 215, 0.25, 0.02, true, 0.8, 0.65, 0.8, 0.65)),
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, 3.1, 5.625, 215, 0.25, 0.02, true, 0.8, 0.65, 0.8, 0.65)));
  }
}
