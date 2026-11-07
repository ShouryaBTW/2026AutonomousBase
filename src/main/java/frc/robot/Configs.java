package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

/** Motor controller configuration for the swerve drivetrain. */
public final class Configs {
  private Configs() {}

  public static final class MAXSwerveModule {
    public static final TalonFXConfiguration drivingConfig = new TalonFXConfiguration();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 12.0 / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      drivingConfig.CurrentLimits.SupplyCurrentLimit = 40;
      drivingConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      drivingConfig.CurrentLimits.StatorCurrentLimit = 70;
      drivingConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      drivingConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      drivingConfig.Feedback.SensorToMechanismRatio = ModuleConstants.kDrivingMotorReduction;
      drivingConfig.Slot0.kP = 0.1;
      drivingConfig.Slot0.kI = 0.0;
      drivingConfig.Slot0.kD = 0.0;
      drivingConfig.Slot0.kV = drivingVelocityFeedForward;
      drivingConfig.Voltage.PeakForwardVoltage = 12.0;
      drivingConfig.Voltage.PeakReverseVoltage = -12.0;

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
      turningConfig.absoluteEncoder
          .inverted(true)
          .positionConversionFactor(turningFactor)
          .velocityConversionFactor(turningFactor / 60.0)
          .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }

    private MAXSwerveModule() {}
  }
}
