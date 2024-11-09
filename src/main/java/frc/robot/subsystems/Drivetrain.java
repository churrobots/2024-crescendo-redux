public class Drivetrain extends SubsystemBase{

    // All the swerve modules.
    // NOTE: in 2023 season, we had an error that caused the swerve module to
    // "twitch" periodically, so here is a reference in case it happens again
    // https://www.chiefdelphi.com/t/vmx-pi-can-spark-max-ids-1-timed-out-while-waiting-for-periodic-status-0/402177/8
    private final RevMAXSwerveModule m_frontLeft = new RevMAXSwerveModule(CANMapping.frontLeftDrivingMotor,
        CANMapping.frontLeftTurningMotor, Constants.kFrontLeftChassisAngularOffset);

    private final RevMAXSwerveModule m_frontRight = new RevMAXSwerveModule(CANMapping.frontRightDrivingMotor,
        CANMapping.frontRightTurningMotor, Constants.kFrontRightChassisAngularOffset);

    private final RevMAXSwerveModule m_rearLeft = new RevMAXSwerveModule(CANMapping.rearLeftDrivingMotor,
        CANMapping.rearLeftTurningMotor, Constants.kRearLeftChassisAngularOffset);

    private final RevMAXSwerveModule m_rearRight = new RevMAXSwerveModule(CANMapping.rearRightDrivingMotor,
        CANMapping.rearRightTurningMotor, Constants.kRearRightChassisAngularOffset);


}
