package team1403.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * This class holds attributes for the robot configuration.
 *
 * <p>
 * The RobotConfig is broken out into different areas,
 * each of which is captured in a class for that area. Each
 * subsystem has its own independent config.
 *
 * <p>
 * The "electrical" configs are treated separate and independent
 * to make it easier to see how the robot should be wired and see
 * any conflicts since these ports specify their config together.
 */
public class Constants {

  // Variables to used by all subsystems.
  public static final double kLoopTime = 0.02;
  //controls if the debug tab is used on shuffleboard
  public static final boolean DEBUG_MODE = false;
  public static final boolean ENABLE_SYSID = false;
  //controls if the debug tab is used on shuffleboard

  /**
   * Swerve Constants.
   * 
   */
  public static class Swerve {
    public static final int kStatusFrameGeneralPeriodMs = 250;
    public static final int kCanTimeoutMs = 250;

    public static final int kModuleUpdateRateMs = 10;
    public static final double kModuleUpdateRateHz = 1.0 / Units.millisecondsToSeconds(kModuleUpdateRateMs);

    public static final double kWheelWidth = Units.inchesToMeters(23);
    public static final double kWheelLength = Units.inchesToMeters(24);
    public static final double kDriveBase = Math.hypot(Swerve.kWheelWidth / 2.0, Swerve.kWheelLength / 2.0);

    public static final Translation2d[] kModulePositions = new Translation2d[]
    {
      // Front left
      new Translation2d(kWheelLength / 2.0, kWheelWidth / 2.0),
      // Front right
      new Translation2d(kWheelLength / 2.0, -kWheelWidth / 2.0),
      // Back left  
      new Translation2d(-kWheelLength / 2.0, kWheelWidth / 2.0),
      // Back right
      new Translation2d(-kWheelLength / 2.0, -kWheelWidth / 2.0)
    };

    public static final int kNumSwerveModules = kModulePositions.length;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kModulePositions);

    public static final double frontLeftEncoderOffset = -Math.PI + 0.082834967179;//-Math.PI + 0.082834967179
    public static final double frontRightEncoderOffset = -0.55 - 3.219825673771961 - Math.PI;//-0.55 - 3.219825673771961 - Math.PI
    public static final double backLeftEncoderOffset = -0.026077673394056; //4.743068596142402, -0.026077673394056
    public static final double backRightEncoderOffset = 1.25 - 6.17273869045182 - 2 * Math.PI;//-0.2966 , 1.25 - 6.17273869045182 - 2 * Math.PI
    
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.8); //3.85 monty (actual 3.8), 4 lehigh (actual 3.83), 3.98 worlds (actual 3.87)
    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2.;
    
    public static final double kFirstDriveStage = (15.0 / 45.0);
    public static final double kDriveReduction = (14.0 / 50.0) * (28.0 / 16.0) * kFirstDriveStage; // ~ 1/6.21
    public static final double kDrivePositionConversionFactor = kWheelDiameterMeters * Math.PI * kDriveReduction; //(14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * kWheelDiameterMeters * Math.PI; //0.05215454470665408

    public static final double kSteerReduction = (15.0 / 32.0) * (10.0 / 60.0);

    public static final double kSteerPositionConversionFactor = 2.0 * Math.PI
        * Swerve.kSteerReduction;

    public static final double kNEOMaxRpm = 5676;

    public static final double kMaxSpeed = kNEOMaxRpm * kDrivePositionConversionFactor / 60.0; 

    public static final double kMaxAngularSpeed = (kMaxSpeed / Math.hypot(kWheelWidth / 2.0, kWheelLength / 2.0)); // 11.96207492071159 rad/s

    // max azimuth speed
    public static final AngularVelocity kMaxTurningSpeed = RadiansPerSecond.of(kNEOMaxRpm * kSteerPositionConversionFactor / 60.0); // theoretical: kNEOMaxRpm * kSteerPositionConversionFactor / 60.0 (~50 rad/s)

    // IMU has an angular velocity, so to get the heading at the right point time add the velocity * a coeff to get the "real" heading
    public static final double kAngVelCoeff = 0.08; //TODO: needs tuning! (generally ranges from -0.15 to 0.15)
    public static final double kCouplingRatio = kFirstDriveStage * kDrivePositionConversionFactor / (2 * Math.PI); //TODO: check this!

    public static final double kVoltageSaturation = 12.0;
    public static final int kDriveCurrentLimit = 45;
    public static final int kSteerCurrentLimit = 25;

    //swerve drive motor
    public static final double kPDrive = 0.1; //0.1
    public static final double kIDrive = 0.0;
    public static final double kDDrive = 0.0;
    public static final double kSDrive = 0.11331; //tune using sysid (volts)
    public static final double kVDrive = 2.5702; //volts instead of % duty cycle (tune with sysid)
    public static final double kADrive = 0.35086; //tune with sysid (volts)

    //swerve module azimuth
    public static final double kPTurning = 0.7; //0.7
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.11;
    public static final double kSTurning = 0.0;

    //front-to-back-disp = ~8.568 inches 
    //left-to-right-disp = 0 inches
    //top-to-bottom disp = 17.82426 inches
    public static final Rotation3d kCameraRotation = new Rotation3d(Math.PI, Units.degreesToRadians(-25), Math.PI);
    public static final Rotation3d kLimelightRotation = new Rotation3d(0, Units.degreesToRadians(-25), Math.PI);
    public static final Translation3d kCameraOffset = new Translation3d(Units.inchesToMeters(-8.568),0,Units.inchesToMeters(17.82426-48));
    public static final Transform3d kCameraTransfrom = new Transform3d(kCameraOffset, kCameraRotation);
    public static final Transform3d kLimelightTransform = new Transform3d(kCameraOffset, kLimelightRotation);
  }

  public static class PathPlanner {
    
    public static final PIDConstants kTranslationPID = new PIDConstants(5.6, 0, 0);
    public static final PIDConstants kRotationPID = new PIDConstants(2.8, 0, 0);
    public static final PathConstraints kPathConstraints = new PathConstraints(Swerve.kMaxSpeed, 3, Swerve.kMaxAngularSpeed, 5);
    public static final PathConstraints kAutoAlignConstraints = new PathConstraints(Swerve.kMaxSpeed, 6, Swerve.kMaxAngularSpeed, 5);
  }

  public static class RioPorts {
    public static final int kArmAbsoluteEncoder = 0;
    public static final int kwristAbsoluteEncoder = 1; // DIO

    public static final int intakePhotogate1 = 3;
    
    public static final int shooterPhotogate = 2;

  }

  /**
   * Configures the CAN bus. These are grouped together
   * rather than by subsystem to more easily detect conflict
   * and understand overall wiring.
   */  
  public static class CanBus {

    private static final int kTBD = 0;

    // Swerve CanBus ids
    public static final int frontLeftDriveID = 13;
    public static final int frontLeftSteerID = 12;
    public static final int frontLeftEncoderID = 22;

    public static final int frontRightDriveID = 9;
    public static final int frontRightSteerID = 8;
    public static final int frontRightEncoderID = 20;

    public static final int backLeftDriveID = 11;
    public static final int backLeftSteerID = 10;
    public static final int backLeftEncoderID = 21;

    public static final int backRightDriveID = 7;
    public static final int backRightSteerID = 6;
    public static final int backRightEncoderID = 23;

    // other
    public static final int powerDistributionID = 60;
    
    public static final int rightPivotMotorID = 5;
    public static final int leftPivotMotorID = 14;

    public static final int shooterMotorTopID = 2;
    public static final int shooterMotorBottomID = 1;
    public static final int intakeMotorID = 4;
    
    public static final int wristMotorID = 15;
  }

  /**
   * Config parameters for tuning the operator interface.
   */
  public static class Operator {

    /**
     * The joystick port for the operator's controller.
     */
    public static final int pilotPort = 0;

    /**
     * Encoder ticks from center still considered close enough to be at center.
     */
    public static final double seekCenterTolerance = 10.0;
  }

  /**
   * Config parameters for tuning the operator interface.
   */
  public static class Driver {

    /**
     * The joystick port for the operator's controller.
     */
    public static final int pilotPort = 1;

    /**
     * Encoder ticks from center still considered close enough to be at center.
     */
    public static final double seekCenterTolerance = 10.0;
  }

  public static class Vision {
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final boolean kExtraVisionDebugInfo = true;
  }

  public static class IntakeAndShooter {
    public static final double kFrameAngle = 250.24629;
    public static final double kFrameClearanceAngle = 234.5; // cone angle
    public static final double kHorizonAngle = 210; 
    public static final double kSpeedReduction = 2.0; // test value
    public static double kStageLineRPM = 5000; //To test
    public static double kCenterLineRPM = 6000;
    public static double kLaunchpadRPM = 5000;
    public static double kFeedShotRPM = 4000;
    public static final double kCloseRPM = 4800;
    public static final double kExpelDeadzone = 0.15;
    public static final int kIntakeCurrentLimit = 40;
  }

  public static class Arm {
    // all angles are in degrees
    public static final double KPArmPivot = 0.0135;
    public static final double KIArmPivot = 0.0;
    public static final double KDArmPivot = 0;
    public static final double kAbsolutePivotOffset = 0;
    public static final double kFeedforwardG = 0.03;
    public static final double kFeedforwardV = 0.0001;

    public static final double kMaxPivotAngle = 230;//180
    public static final double kMinPivotAngle = 75;
    public static final double kPivotMotorMaxAmperage = 40;

    public static final int kPivotMotorCurrentLimit = 30;
    public static final double kPivotMotorVoltageLimit = 12;

    public static double kIntakeSetpoint = 92; // 92
    public static  double kAmpSetpoint = 210;
    public static  double kLoadingSetpoint = 150; //150
    public static  double kDriveSetpoint = 114;
    public static  double kDefaultClose = 114;
  }

  public static class Wrist {
    public static final double kWristConversionFactor = 0;
    public static final double kAbsoluteWristOffset = 0;

    public static final double KPWrist = 0.0097; //original value 0.0092 changed - 0.0097
    public static final double KIWrist = 0.0000;
    public static final double KDWrist = 0;

    public static final double kTopLimit = 180;
    public static final double kBottomLimit = 0;

    public static  double kIntakeSetpoint = 142;
    public static  double kAmpSetpoint = 160.5;
    public static double kAmpShoootingSetpoint = 142;
    public static  double kLoadingSetpoint = 90;
    public static  double kDriveSetpoint = 140;//140
    public static  double kDefaultClose = 136;
    public static double kStageLineSetpoint = 138;//To test
    public static double kStageLineSideSetpoint = 135;//136 version 2
    public static double kSideLineSourceSetpoint = 130; 
    public static double kLaunchpadSetpoint = 140;
    public static double kCenterLineSetpoint = 133;//115
    public static  double kShootingAngle = 147;//147 for teleop working


    public static final double kWristUpperLimit = 150;
    public static final double kWristLowerLimit = 130;
    public static final double kWristConstraint = 140;
    public static final double kArmConstraint = 120;
   //Algae intake speed for Arjuna practicing
    public static final double motorSpeed = 0.2;
    
  }
}