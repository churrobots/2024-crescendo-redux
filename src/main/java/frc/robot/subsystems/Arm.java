

/** A robot arm subsystem that moves with a motion profile. */
public class Arm extends TrapezoidProfileSubsystem {

    class Constants {
      // TODO: are these values descriptive or prescriptive?
      static final double kMaxVelocityRadPerSecond = 3;
      static final double kMaxAccelerationRadPerSecSquared = 3;
      static final TrapezoidProfile.Constraints trapezoidProfile = new TrapezoidProfile.Constraints(
          Constants.kMaxVelocityRadPerSecond, Constants.kMaxAccelerationRadPerSecSquared);
  
      // The offset of the arm from the horizontal in its neutral position,
      // measured from the horizontal.
      static final double kArmOffsetRads = 0;
  
      // These are all the constants from the SparkMAX demo code.
      static final TunableDouble kP = new TunableDouble("armKP", 4.6);
      static final TunableDouble kI = new TunableDouble("armKI", 0);
      static final TunableDouble kD = new TunableDouble("armKD", 2);
      static final double kIz = 0;
    //   static final double kFF = 0;
      // TODO: what are the units of these values? 1.0 = 100%?
      static final double kMaxOutput = 0.9;
      static final double kMinOutput = -0.7;
  
      // These are all the constants from the sample WPIlib trapezoid subsystem code.
      // TODO: double check these definitions.
      // SVolts = static gain = voltage required to initially move the system from rest.
      static final TunableDouble kSVolts = new TunableDouble("feedFowardSVolts", 0);
      // GVolts = gravity gain = voltage required to offset the downward force of gravity.
      static final TunableDouble kGVolts = new TunableDouble("feedFowardGVolts", 0.47);
      // kVVolts = velocity gain = voltage required to maintain a 1rad velocity?
      static final TunableDouble kVVoltSecondPerRad = new TunableDouble("feedFowardVVoltSecondPerRad", 2.92);
      // kAVolts = acceleration gain = voltage required to accelerate by 1rad/sec?
      static final TunableDouble kAVoltSecondSquaredPerRad = new TunableDouble("feedFowardAVoltSecondSquaredPerRad",
          0.02);
  
      // These constants are from the sample WPIlib code and shouldn't need to change.
      static final int kCPR = 8192;
  
      // Default slot should be fine according to:
      // https://www.chiefdelphi.com/t/sparkmax-pid-controller/427438/4
      static final int defaultPidSlot = 0;
  
      // TODO: what are the units of these values?
      static final TunableDouble groundPosition = new TunableDouble("groundPosition", 0); // tune
      static final TunableDouble ampPosition = new TunableDouble("ampPosition", 0.15); // tune
      static final TunableDouble speakerPosition = new TunableDouble("speakerPosition", 0.2); // tune
      static final TunableDouble defaultPosition = new TunableDouble("defaultPosition", 0); // tune
    }

    // Arm has two motors.
    final CANSparkMax right_motor = new CANSparkMax(CANMapping.rightArmMotor, MotorType.kBrushless);
    final CANSparkMax left_motor = new CANSparkMax(CANMapping.leftArmMotor, MotorType.kBrushless);

  final SparkPIDController m_pidController;

  // Slew rate is the change of voltage per unit of time.
  // In this case, the voltage is the input to the motors.
  // Limiting this is useful because jerky inputs might damage the arm.
  final SlewRateLimiter rateLimiter = new SlewRateLimiter(12 /* volts per second */);

  // Use a feedforward control system to factor in physical factors
  final ArmFeedforward m_feedforward = new ArmFeedforward(
      Constants.kSVolts.get(),
      Constants.kGVolts.get(),
      Constants.kVVoltSecondPerRad.get(),
      Constants.kAVoltSecondSquaredPerRad.get());

      /**
   * An alternate encoder object is constructed using the GetAlternateEncoder()
   * method on an existing CANSparkMax object. If using a REV Through Bore
   * Encoder, the type should be set to quadrature and the counts per
   * revolution set to 8192
   */
  // TODO: figure out type of encoder
  final SparkAbsoluteEncoder m_absoluteEncoder;

  /** Create a new ArmSubsystem. */
  // We used some sample code from:
  // https://github.com/Delmar-Robotics-Engineers-At-MADE/2024-Robot/blob/main/src/main/java/frc/robot/subsystems/Arm.java
  public Arm() {
    super(Constants.trapezoidProfile, Constants.kArmOffsetRads);

    right_motor.restoreFactoryDefaults();
    right_motor.setSmartCurrentLimit(40);
    right_motor.setIdleMode(IdleMode.kBrake);
    right_motor.setInverted(false);
    m_absoluteEncoder = right_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    m_pidController = right_motor.getPIDController();
    m_pidController.setFeedbackDevice(m_absoluteEncoder);
    // TODO: do we need to enable PID wrapping?
    // m_pidController.setPositionPIDWrappingEnabled(true);

    left_motor.restoreFactoryDefaults();
    left_motor.setSmartCurrentLimit(40);
    left_motor.setIdleMode(IdleMode.kBrake);
    left_motor.setInverted(true);
    left_motor.follow(right_motor, true);

    // TODO: tune the PID for the arm
    m_pidController.setP(Constants.kP.get());
    m_pidController.setI(Constants.kI.get());
    m_pidController.setD(Constants.kD.get());
    m_pidController.setIZone(Constants.kIz);
    // m_pidController.setFF(Constants.kFF);
    m_pidController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);

    enable();
    // Programming motors
    right_motor.burnFlash();
    left_motor.burnFlash();
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    double feedforward = m_feedforward.calculate(
        setpoint.position * 2 * Math.PI,
        setpoint.velocity);
    m_pidController.setReference(
        setpoint.position,
        CANSparkMax.ControlType.kPosition,
        Constants.defaultPidSlot,
        feedforward);
    SmartDashboard.putNumber("Arm:mostRecentSetpointPosition", setpoint.position);
    SmartDashboard.putNumber("Arm:mostRecentFeedForward", feedforward);
  }

  // Move arm to speaker shot angle.
  public void move_speaker() {
    moveTo(0.048);
  }

  void moveTo(double percentageRotation) {
    double position = m_absoluteEncoder.getPosition();
    if (position > 0.8) {
      position = 0;
    }
    if (position < 0.005) {
      // Workaround the encoder sometimes going negative
      disable();
      right_motor.set(0.1);
    } else {
      // Now encoder is for sure positive, so we can do normal position control
      enable();
      setGoal(percentageRotation);
    }
  }

  public void move_amp() {
    moveTo(0.28);
  }

  public void move_Eject() {
    moveTo(.16); // or .125
  }

  public void move_mid() {
    moveTo(0.1025); //.1075 was the new one, .1025 was the OG one which we're trying now at the practice field
  }

  public void move_Default() {
    disable();
    double position = m_absoluteEncoder.getPosition();
    if (position > 0.8) {
      position = 0;
    }
    if (position > 0.12) {
      right_motor.set(-0.4);
    } else if (position > 0.08) {
      right_motor.set(-0.3);
    } else if (position > 0.03) {
      right_motor.set(-0.05);
    } else {
      right_motor.stopMotor();
    }
  }

  public boolean armIsHigh() {
    double position = m_absoluteEncoder.getPosition();
    if (position > 0.15) {
      return true;
    } else
      return false;

  }

  void _debug() {
    SmartDashboard.putNumber("Arm:encoder", m_absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Arm:appliedOutputRight", right_motor.getAppliedOutput());
    SmartDashboard.putNumber("Arm:appliedOutputLeft", left_motor.getAppliedOutput());
  }

  @Override
  public void periodic() {
    super.periodic();
    // TODO: uncomment to see debug info for tuning the arm
    // _debug();
  }

}