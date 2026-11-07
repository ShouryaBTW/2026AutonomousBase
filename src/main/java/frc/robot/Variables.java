package frc.robot;

/** Shared Limelight pose data used to correct drivetrain odometry. */
public final class Variables {
  private Variables() {}

  public static final class limelight {
    public static double ll_x = 0;
    public static double ll_y = 0;
    public static double ll_rot = 0;
    public static boolean hasTarget = false;

    private limelight() {}
  }
}
