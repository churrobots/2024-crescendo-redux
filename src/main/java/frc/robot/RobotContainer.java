// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class RobotContainer {

    static final class Constants {
        public static final int kdriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kDriveDeadband = 0.1;
        public static final double kSlowDriveScaling = 0.25;
        public static final double kSuperSlowDriveScaling = 0.15;

        // Whether the driver wants to use the Logitech X3D flightstick controller.
        // If false, assume driver is using an Xbox controller.
        public static final boolean kDriverUsesLogitech = false;
    }

    // Logitech flightstick controller button and joystick axes assignments.
    // Driver uses this controller if Constants.kDriverUsesLogitech is true.
    final LogitechX3D driverLogitech = new LogitechX3D(Constants.kDriverControllerPort);
    final Trigger button1Trigger = new JoystickButton(driverLogitech, 1); // trigger
    final Trigger button2Trigger = new JoystickButton(driverLogitech, 2); // side thumb button
    final Trigger button3Trigger = new JoystickButton(driverLogitech, 3); // hook up
    final Trigger button5Trigger = new JoystickButton(driverLogitech, 5); // hook down
    final Trigger button7Trigger = new JoystickButton(driverLogitech, 7); // numbered buttons...
    final Trigger button8Trigger = new JoystickButton(driverLogitech, 8);
    final Trigger button9Trigger = new JoystickButton(driverLogitech, 9);
    final Trigger button10Trigger = new JoystickButton(driverLogitech, 10);
    final Trigger button11Trigger = new JoystickButton(driverLogitech, 11);
    final Trigger button12Trigger = new JoystickButton(driverLogitech, 12);

    final DoubleSupplier forwardAxisLogitech = driverLogitech::getY;
    final DoubleSupplier sidewaysAxisLogitech = driverLogitech::getX;
    final DoubleSupplier rotationAxisLogitech = driverLogitech::getTwist;
    final DoubleSupplier sliderAxisLogitech = driverLogitech::getThrottle;

    // Xbox controller button and joystick axes assignments.
    // Driver uses this controller if Constants.kDriverUsesLogitech is False.
    final XboxController driverXbox = new XboxController(Constants.kdriverControllerPort);
    final Trigger startButtonDriver = new JoystickButton(driverXbox, Button.kStart.value);
    final Trigger backButtonDriver = new JoystickButton(driverXbox, Button.kStart.value);
    final Trigger leftBumperDriver = new JoystickButton(driverXbox, Button.kLeftBumper.value);
    final Trigger rightBumperDriver = new JoystickButton(driverXbox, Button.kRightBumper.value);
    final Trigger startAndBackButtonDriver = new Trigger(() -> {
        return startButtonDriver.getAsBoolean() && backButtonDriver.getAsBoolean();
    });
    final Trigger aButtonDriver = new JoystickButton(driverXbox, Button.kA.value);
    final Trigger bButtonDriver = new JoystickButton(driverXbox, Button.kB.value);
    final Trigger yButtonDriver = new JoystickButton(driverXbox, Button.kY.value);
    final Trigger xButtonDriver = new JoystickButton(driverXbox, Button.kX.value);
    final Trigger rightjoyAnalogTrigger = new Trigger(() -> {
    boolean triggerIsPressedEnough = driverXbox.getRightTriggerAxis() > 0.28;
        return triggerIsPressedEnough;
    });
    final Trigger leftJoyAnalogTrigger = new Trigger(() -> {
        boolean triggerIsPressedEnough = driverXbox.getLeftTriggerAxis() > 0.28;
        return triggerIsPressedEnough;
    });

    final DoubleSupplier forwardAxisXbox = driverXbox::getLeftY;
    final DoubleSupplier sidewaysAxisXbox = driverXbox::getLeftX;
    final DoubleSupplier rotationAxisXbox = driverXbox::getRightX;

    // Operator Xbox controller.
    final XboxController operatorXbox = new XboxController(Constants.kOperatorControllerPort);
    final Trigger leftBumperOperator = new JoystickButton(operatorXbox, Button.kLeftBumper.value);
    final Trigger aButtonOperator = new JoystickButton(operatorXbox, Button.kA.value);
    final Trigger xButtonOperator = new JoystickButton(operatorXbox, Button.kX.value);
    final Trigger bButtonOperator = new JoystickButton(operatorXbox, Button.kB.value);
    final Trigger yButtonOperator = new JoystickButton(operatorXbox, Button.kY.value);
    final Trigger startButtonOperator = new JoystickButton(operatorXbox, Button.kStart.value);
    final Trigger backButtonOperator = new JoystickButton(operatorXbox, Button.kBack.value);
    final Trigger povUpOperator = new POVButton(operatorXbox, 0);
    final Trigger povDownOperator = new POVButton(operatorXbox, 180);
    final Trigger leftjoyTrigger = new JoystickButton(operatorXbox, Button.kLeftStick.value);
    final Trigger rightjoyTrigger = new JoystickButton(operatorXbox, Button.kRightStick.value);

    // Variables that will be set to either xbox or logitech axes in
    // configureButtonBindings().
    DoubleSupplier driverForwardAxis;
    DoubleSupplier driverSidewaysAxis;
    DoubleSupplier driverRotationAxis;

}