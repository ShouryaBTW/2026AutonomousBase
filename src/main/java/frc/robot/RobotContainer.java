package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoRoutines;
import frc.robot.auto.AutoSelector;
import frc.robot.subsystems.DriveSubsystem;

/** Wires the drivetrain to the autonomous mode selector and route builder. */
public class RobotContainer {
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final AutoSelector autoSelector = new AutoSelector();
  private final AutoRoutines autoRoutines = new AutoRoutines(m_robotDrive);

  public RobotContainer() {
    autoSelector.publishToDashboard();
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
