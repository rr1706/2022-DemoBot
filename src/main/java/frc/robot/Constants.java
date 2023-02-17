package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Static method containing all constant values for the robot in one location
 */
public final class Constants {

  public static final class CurrentLimit {
    public static final int kTranslation = 50;
    public static final int kRotation = 30;
  }

  public static final class GoalConstants {
    public static final Translation2d kGoalLocation = new Translation2d(8.23, 4.115);
    public static final Translation2d kWrongBallGoal = new Translation2d(5.50, 4.115);
    public static final Translation2d kHangerLocation = new Translation2d(2.00, 6.00);

  }

  /**
   * Static method containing all Drivetrain constants
   */
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 3; // CANID of the Translation SparkMAX
    public static final int kFrontRightDriveMotorPort = 7; // CANID of the Translation SparkMAX
    public static final int kBackLeftDriveMotorPort = 1; // CANID of the Translation SparkMAX
    public static final int kBackRightDriveMotorPort = 5; // CANID of the Translation SparkMAX

    public static final int kFrontLeftTurningMotorPort = 4; // CANID of the Rotation SparkMAX
    public static final int kFrontRightTurningMotorPort = 8; // CANID of the Rotation SparkMAX
    public static final int kBackLeftTurningMotorPort = 2; // CANID of the Rotation SparkMAX
    public static final int kBackRightTurningMotorPort = 6; // CANID of the Rotation SparkMAX

    public static final int kFrontLeftTurningEncoderPort = 6; // Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 7; // Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 4; // Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 5; // Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = -0.954-2.949+0.121; // Encoder Offset in Radians
    public static final double kFrontRightOffset = -2.857-3.339; // Encoder Offset in Radians
    public static final double kBackLeftOffset = 2.784-8.335; // Encoder Offset in Radians
    public static final double kBackRightOffset = -0.967-2.403; // Encoder Offset in Radians

    // Drive motor PID is best done on the roboRIO currently as the SparkMAX does
    // not allow for static gain values on the PID controller,
    // these are necessary to have high accuracy when moving at extremely low RPMs
    // public static final double[] kFrontLeftTuningVals = {0.0120,0.2892,0.25,0};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    // public static final double[] kFrontRightTuningVals = {0.0092,0.2835,0.25,1};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    // public static final double[] kBackLeftTuningVals = {0.0142,0.2901,0.25,2};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    // public static final double[] kBackRightTuningVals = {0.0108,0.2828,0.25,3};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}

    public static final double[] kFrontLeftTuningVals = { 0.0150*0.5, 0.2850*0.97, 0.15, 0 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}
    public static final double[] kFrontRightTuningVals = { 0.0150*0.5, 0.2850*0.97, 0.1, 1 }; // {Static Gain, FeedForward,
                                                                                      // Proportional Gain, ModuleID for
                                                                                      // Tuning}
    public static final double[] kBackLeftTuningVals = { 0.0150*0.5, 0.2850*0.97, 0.15, 2 }; // {Static Gain, FeedForward,
                                                                                    // Proportional Gain, ModuleID for
                                                                                    // Tuning}
    public static final double[] kBackRightTuningVals = { 0.0150*0.5, 0.2850*0.97, 0.15, 3 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}

    // NOTE: 2910 Swerve the wheels are not directly under the center of rotation
    // (Take into consideration when measuring)
    public static final double kWheelBaseWidth = 0.4064; // Center distance in meters between right and left wheels on
                                                         // robot
    public static final double kWheelBaseLength = 0.3556; // Center distance in meters between front and back wheels on
                                                          // robot

    // Because the swerve modules poisition does not change, define a constant
    // SwerveDriveKinematics for use throughout the code
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2),
        new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2));

    public static final double kMaxAcceleration = 3.0;
    public static final double kMaxSpeedMetersPerSecond = 3.25*1.333*1.204; // Maximum Sustainable Drivetrain Speed under Normal
                                                                // Conditions & Battery, Robot will not exceed this
                                                                // speed in closed loop control
    public static final double kMaxAngularSpeed = 1.5*Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = 1.5*Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly

    public static final double kInnerDeadband = 0.10; // This value should exceed the maximum value the analog stick may
                                                      // read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                      // when aimed at a 45deg angle (Such that X and Y are are
                                                      // maximized simultaneously)
    public static final double kTranslationSlew = 1.500;
    public static final double kRotationSlew = 3.75;

    // Minimum allowable rotation command (in radians/s) assuming user input is
    // squared using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed
        * Math.pow(DriveConstants.kInnerDeadband, 2);
    // Minimum allowable tranlsation command (in m/s) assuming user input is squared
    // using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond
        * Math.pow(DriveConstants.kInnerDeadband, 2);

    public static final double[] kKeepAnglePID = { 0.300, 0, 0 }; // Defines the PID values for the keep angle PID

  }

  /**
   * Static method containing all Swerve Module constants
   */
  public static final class ModuleConstants {
    public static final double kTranslationRampRate = 4.0; // Units of %power/s, ie 4.0 means it takes 0.25s to reach
                                                           // 100% power from 0%
    private static final double kTranslationGearRatio = 8.33333333*0.750*0.830769231; // Overall gear ratio of the swerve module
    private static final double kWheelDiameter = 0.0942; // Wheel Diameter in meters, may need to be
                                                                         // experimentally determined due to compliance
                                                                         // of floor/tread material

    public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI; // Calculates
                                                                                                                  // the
                                                                                                                  // conversion
                                                                                                                  // factor
                                                                                                                  // of
                                                                                                                  // RPM
                                                                                                                  // of
                                                                                                                  // the
                                                                                                                  // translation
                                                                                                                  // motor
                                                                                                                  // to
                                                                                                                  // m/s
                                                                                                                  // at
                                                                                                                  // the
                                                                                                                  // floor

    // NOTE: You shoulds ALWAYS define a reasonable current limit when using
    // brushless motors
    // due to the extremely high stall current avaialble

    public static final double[] kTurnPID = { 0.600, 0, 0 }; // Defines the PID values for rotation of the serve
                                                             // modules, should show some minor oscillation when no
                                                             // weight is loaded on the modules
  }

  /**
   * Static method containing all User I/O constants
   */
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0; // When making use of multiple controllers for drivers each
                                                       // controller will be on a different port
    public static final int kOperatorControllerPort = 1; // When making use of multiple controllers for drivers each
                                                         // controller will be on a different port
  }

  /**
   * Static method containing all Global constants
   */
  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final double kLoopTime = 0.020;
  }

  public static final class ShooterConstants {
    public static final int kFeedID = 9;
    public static final int kShootID = 10; // CANID of the Motor Controller for the Shooter Motor
  }

  /**
   * Static method containing all Autonomous constants
   */
  public static final class AutoConstants {
    public static final double kMaxAcceleration = 2.50;
    public static final double kMaxSpeed = 3.25; // Maximum Sustainable Drivetrain Speed under Normal Conditions &
                                                 // Battery, Robot will not exceed this speed in closed loop control
    public static final double kMaxAngularSpeed = 1.25*Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = 1.25*Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kPXController = 3.0;
    public static final double kPYController = 3.0;
    public static final double kPThetaController = 3.0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed, kMaxAngularAccel); // Creates a trapezoidal motion for the auto rotational commands
  }
}