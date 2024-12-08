// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.churrolib.ChurroSim;
import frc.churrolib.RevMAXSwerveModuleSim;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

/**
 * Build on top of the Rev MAX sample code
 * https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java
 * More RevLib examples:
 * https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java
 */
public class RevMAXSwerveModule {

  private static final class Constants {

    // Tuning values
    public static final double kWheelDiameterMeters = 0.0831;

    public static final double kDrivingP = .08;// .04
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;

    public static final double kTurningP = 2;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    // Note that the module itself has 45 teeth on the wheel's bevel gear, 22 teeth
    // on the first-stage spur gear, 15 teeth on the bevel pinion.
    public static final int kDrivingMotorPinionTeeth = 16;
    public static final int kWheelBevelTeeth = 45;
    public static final int kFirstStageSpurTeeth = 20;
    public static final int kBevelPinionTeeth = 15;

    public static final double kMotorFreeSpeedRpm = 5676; // Neo motors are 5676 max RPM
  }

  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  // Invert the turning encoder, since the output shaft rotates in the opposite
  // direction of
  // the steering motor in the MAXSwerve Module.
  private final boolean kTurningEncoderInverted = true;

  // Turning motor reduction from "Azimuth Ratio" of MAXSwerve module spec:
  // https://www.revrobotics.com/rev-21-3005/
  private static final double kTurningMotorReduction = 9424 / 203;

  // Calculations required for driving motor conversion factors and feed forward
  private final double kDrivingMotorFreeSpeedRps = Constants.kMotorFreeSpeedRpm / 60;

  private final double kWheelCircumferenceMeters = Constants.kWheelDiameterMeters * Math.PI;

  private final double kDrivingMotorReduction = ((double) Constants.kWheelBevelTeeth * Constants.kFirstStageSpurTeeth)
      / ((double) Constants.kDrivingMotorPinionTeeth * Constants.kBevelPinionTeeth);
  private final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;

  private final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters
      / kDrivingMotorReduction; // meters
  private final double kDrivingEncoderVelocityFactor = (kWheelCircumferenceMeters
      / kDrivingMotorReduction) / 60.0; // meters per second because native units are RPM

  private final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
  private final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
                                                                             // because native units are RPM

  private final double kTurningEncoderPositionPIDMinInput = 0; // radians
  private final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

  private final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
  private final double kDrivingMinOutput = -1;
  private final double kDrivingMaxOutput = 1;

  private final double kTurningFF = 0;
  private final double kTurningMinOutput = -1;
  private final double kTurningMaxOutput = 1;

  private final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
  private final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

  private final int kDrivingMotorCurrentLimit = 50; // amps
  private final int kTurningMotorCurrentLimit = 20; // amps

  final RevMAXSwerveModuleSim m_sim;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public RevMAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    SparkMaxConfig drivingConfig = new SparkMaxConfig();
    SparkMaxConfig turningConfig = new SparkMaxConfig();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();
    drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    drivingConfig.encoder.positionConversionFactor(kDrivingEncoderPositionFactor);
    drivingConfig.encoder.velocityConversionFactor(kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs. Note that we have to set the conversion factor on BOTH the native
    // encoder and the absolute encoder (even though we're theoretically only
    // using the absolute encoder), since SparkSim relies on the native encoder
    // conversion config when iterating the sim.
    turningConfig.absoluteEncoder.positionConversionFactor(kTurningEncoderPositionFactor);
    turningConfig.absoluteEncoder.velocityConversionFactor(kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    turningConfig.absoluteEncoder.inverted(kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningConfig.closedLoop.positionWrappingEnabled(true);
    turningConfig.closedLoop.positionWrappingMinInput(kTurningEncoderPositionPIDMinInput);
    turningConfig.closedLoop.positionWrappingMaxInput(kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    drivingConfig.closedLoop.p(Constants.kDrivingP);
    drivingConfig.closedLoop.i(Constants.kDrivingI);
    drivingConfig.closedLoop.d(Constants.kDrivingD);
    drivingConfig.closedLoop.velocityFF(kDrivingFF);
    drivingConfig.closedLoop.outputRange(kDrivingMinOutput,
        kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    turningConfig.closedLoop.p(Constants.kTurningP);
    turningConfig.closedLoop.i(Constants.kTurningI);
    turningConfig.closedLoop.d(Constants.kTurningD);
    turningConfig.closedLoop.velocityFF(kTurningFF);
    turningConfig.closedLoop.outputRange(kTurningMinOutput,
        kTurningMaxOutput);

    drivingConfig.idleMode(kDrivingMotorIdleMode);
    turningConfig.idleMode(kTurningMotorIdleMode);
    drivingConfig.smartCurrentLimit(kDrivingMotorCurrentLimit);
    turningConfig.smartCurrentLimit(kTurningMotorCurrentLimit);

    // This identifies the calibrated zero of the absolute encoder, which
    // by default is each 90deg offset from each other if you use the
    // plastic calibration templates that Rev gave us with the modules.
    // TODO: revisit and calibrate with all wheels facing forward?
    m_chassisAngularOffset = chassisAngularOffset;

    // Accept the initial state of the wheels and turning motors, so that
    // the rest of the code knows where it's starting from.
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);

    // Apply the configuration we just built out. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state.
    // Persist the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Start at zero so that simulation doesn't freak out. Otherwise
    // it will be trying to simulate without a starting Control Mode.
    m_drivingPIDController.setReference(0, SparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(0, SparkMax.ControlType.kPosition);

    // The sim needs to be initialized in the constructor because the CAN IDs are
    // passed in and we won't have the SparkMax references until this point.
    m_sim = new RevMAXSwerveModuleSim(
        m_drivingSparkMax,
        kDrivingMotorReduction,
        kDrivingEncoderVelocityFactor,
        m_turningSparkMax,
        kTurningMotorReduction,
        kTurningEncoderVelocityFactor);
    ChurroSim.register(m_sim);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    // This is supposed to track the un-corrected state. Otherwise you end up with
    // incorrect angle values that mismatch the driving velocity values.
    m_desiredState = desiredState;
  }
}