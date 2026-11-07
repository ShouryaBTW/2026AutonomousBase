package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelector {
  private final SendableChooser<AutoMode> chooser = new SendableChooser<>();

  public AutoSelector() {
    chooser.setDefaultOption(AutoMode.RED_RIGHT.getLabel(), AutoMode.RED_RIGHT);
    chooser.addOption(AutoMode.RED_LEFT.getLabel(), AutoMode.RED_LEFT);
    chooser.addOption(AutoMode.BLUE_RIGHT.getLabel(), AutoMode.BLUE_RIGHT);
    chooser.addOption(AutoMode.BLUE_LEFT.getLabel(), AutoMode.BLUE_LEFT);
  }

  public void publishToDashboard() {
    SmartDashboard.putData("Auto Selector", chooser);
  }

  public AutoMode getSelectedMode() {
    AutoMode selectedMode = chooser.getSelected();
    return selectedMode != null ? selectedMode : AutoMode.RED_RIGHT;
  }

  public double getSelectedStartingHeadingDeg() {
    return getSelectedMode().getStartingHeadingDeg();
  }

  public double getSelectedStartingX() {
    return getSelectedMode().getStartingX();
  }

  public double getSelectedStartingY() {
    return getSelectedMode().getStartingY();
  }
}
