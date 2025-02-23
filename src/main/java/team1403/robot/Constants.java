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
  public static final boolean ENABLE_SYSID = true;

  public static class PathPlanner {
  }

  public static class RioPorts {
    public static final int WristEncoderID = 0;
    public static final int algaeIntakePhotogateID = 0;

  }

  /**
   * Configures the CAN bus. These are grouped together
   * rather than by subsystem to more easily detect conflict
   * and understand overall wiring.
   */  
  public static class CanBus {

    
    public static final int powerDistributionID = 42;
    public static final int intakeMotorID = 1;
    public static final int CANRangeID = 27;
    public static final int wristMotorID = 3;


    public static final int leftElevatorMotorID = 10;
    public static final int rightElevatorMotorID = 11;
    public static final int elbowMotorID = 1;
    public static final int algaeIntakeMotorID = 4;

    public static final int pigeon2ID = 2;

    // Swerve CanBus ids
    public static final int frontLeftDriveID = 12;
    public static final int frontLeftSteerID = 15;
    public static final int frontLeftEncoderID = 23;

    public static final int frontRightDriveID = 8;
    public static final int frontRightSteerID = 2;
    public static final int frontRightEncoderID = 22;

    public static final int backLeftDriveID = 16;
    public static final int backLeftSteerID = 13;
    public static final int backLeftEncoderID = 24;

    public static final int backRightDriveID = 5;
    public static final int backRightSteerID = 2;
    public static final int backRightEncoderID = 21;
  }

  /**
   * Config parameters for tuning the operator interface.
   */
  public static class Operator {

    /**
     * The joystick port for the operator's controller.
     */
    public static final int pilotPort = 0;
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
    public static final double algaeEstimateKonstant = 3.8;

    //front-to-back-disp = ~8.568 inches 
    //left-to-right-disp = 0 inches
    //top-to-bottom disp = 17.82426 inches
    public static final Rotation3d kCameraRotation = new Rotation3d(Math.PI, Units.degreesToRadians(-25), Math.PI);
    public static final Rotation3d kLimelightRotation = new Rotation3d(0, Units.degreesToRadians(-25), Math.PI);
    public static final Translation3d kCameraOffset = new Translation3d(Units.inchesToMeters(-8.568),0,Units.inchesToMeters(17.82426-48));
    public static final Transform3d kCameraTransfrom = new Transform3d(kCameraOffset, kCameraRotation);
    public static final Transform3d kLimelightTransform = new Transform3d(kCameraOffset, kLimelightRotation);
  }

  public static class AlgaeIntake {
    public static double upPos = 1;
    public static double downPos = 0;
  }

  public static class Elevator {
    public static final double kPSparkMax = 0.0135;
    public static final double kISparkMax = 0.0;
    public static final double kDSparkMax = 0;
    public static final double kFeedforwardG = 0.01;
    public static final double kFeedforwardV = 0.001;

    public static class Setpoints {
      public static final double L1 = 5;
      public static final double L2 = 10;
      public static final double L3 = 20;
      public static final double L4 = 0;
      public static final double down = 0;
      public static final double source = 0;
    }

    public static class Command {
      public static final double movementUpGain = 3.5;
      public static final double movementDownGain = 3.0;
      public static final double maxSpeed = 90;
      public static final double minSpeed = 5;
      public static final double setPointMargin = 0.5;
      public static final double simPositionFactor = 1; 
      public static final double elevatorUpRampUpTime = 1;
      public static final double elevatorUpRampDownTime = 0.01;
      public static final double elevatorDownRampUpTime = 1;
      public static final double elevatorDownRampDownTime = 0.01;
    }
}

  public static class Wrist {
    public static class Setpoints{
      public static final double L1Setpoint = 0;
      public static final double L2Setpoint = 0;
      public static final double L3Setpoint = 0;
      public static final double L4Setpoint = 0;
    }

    public static final double WristKS = 0;
    public static final double WristKG = 0.51 / 12;
    public static final double WristKV = 0;

    public static final double WristKP = 3.2;//2.7079; //keep testing
    public static final double WristKI = 0;
    public static final double WristKD = 0;//0.69018;

    public static final double maxVelo = 3;
    public static final double maxAccel = 15;

    public static final double WristEncoderOffset = -0.75;

  }

  public static class Climber {
    public static final int leftMotor = 0; //check 
    public static final int rightMotor = 0;
  }
}