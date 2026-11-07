package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.AutoRoutines;
import frc.robot.auto.AutoSelector;
import frc.robot.subsystems.DriveSubsystem;

/** Wires the drivetrain to the autonomous mode selector and route builder. */
public class RobotContainer {
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final XboxController driverController =
      new XboxController(DriveConstants.kDriverControllerPort);
  private final AutoSelector autoSelector = new AutoSelector();
  private final AutoRoutines autoRoutines = new AutoRoutines(m_robotDrive);

  public RobotContainer() {
    autoSelector.publishToDashboard();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    m_robotDrive.setDefaultCommand(
        new RunCommand(
                () -> {
                  // Xbox Y is positive when pushed down; invert it for forward field X.
                  double xSpeed =
                      -MathUtil.applyDeadband(
                          driverController.getLeftY(), DriveConstants.kDriverDeadband);
                  // Positive field Y points left, opposite the Xbox stick's positive-right X.
                  double ySpeed =
                      -MathUtil.applyDeadband(
                          driverController.getLeftX(), DriveConstants.kDriverDeadband);
                  // Positive WPILib rotation is counterclockwise; right-stick right is clockwise.
                  double rotationSpeed =
                      -MathUtil.applyDeadband(
                          driverController.getRightX(), DriveConstants.kDriverDeadband);

                  m_robotDrive.drive(xSpeed, ySpeed, rotationSpeed, true);
                },
                m_robotDrive)
            .finallyDo(interrupted -> m_robotDrive.drive(0.0, 0.0, 0.0, true)));
  }

  public Command getAutonomousCommand() {
    return autoRoutines.getCommandForMode(autoSelector.getSelectedMode());
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  public double getSelectedStartingHeadingDeg() {
    return autoSelector.getSelectedStartingHeadingDeg();
  }

  public double getSelectedStartingX() {
    return autoSelector.getSelectedStartingX();
  }

  public double getSelectedStartingY() {
    return autoSelector.getSelectedStartingY();
  }
}
