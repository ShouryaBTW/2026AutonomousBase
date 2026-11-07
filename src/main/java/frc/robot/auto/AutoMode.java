package frc.robot.auto;

public enum AutoMode {
  // Starting poses are in WPILib/Limelight blue-field coordinates (meters/degrees).
  RED_RIGHT("Red Right", 270.0, 12.2, 7.3),
  RED_LEFT("Red Left", 90.0, 12.2, 0.6),
  BLUE_RIGHT("Blue Right", 90.0, 4.152, 0.7),
  BLUE_LEFT("Blue Left", 270.0, 4.25, 7.4);

  private final String label;
  private final double startingHeadingDeg;
  private final double startingX;
  private final double startingY;
  AutoMode(
      String label,
      double startingHeadingDeg,
      double startingX,
      double startingY) {
    this.label = label;
    this.startingHeadingDeg = startingHeadingDeg;
    this.startingX = startingX;
    this.startingY = startingY;
  }

  public String getLabel() {
    return label;
  }

  public double getStartingHeadingDeg() {
    return startingHeadingDeg;
  }

  public double getStartingX() {
    return startingX;
  }

  public double getStartingY() {
    return startingY;
  }

}
