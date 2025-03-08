package team1403.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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

  public static class RioPorts {
    private static final int kTBD = 0;

    public static final int kAlgaeIntakePhotogateID = kTBD;
  }

  /**
   * Configures the CAN bus. These are grouped together
   * rather than by subsystem to more easily detect conflict
   * and understand overall wiring.
   */  
  public static class CanBus {

    private static final int kTBD = 0;

    // other
    public static final int powerDistributionID = 42;
    public static final int intakeMotorID = 1;
    public static final int CANRangeID = 27;
    public static final int wristMotorID = 3;

    public static final int kCandleID = kTBD;


    public static final int leftElevatorMotorID = 10;
    public static final int rightElevatorMotorID = 11;
    public static final int algaeIntakeMotorID = 4;
    public static final int algaeWristMotorID = kTBD; //tbd
    
    public static final int leftClimberMotor = 18;  
    public static final int rightClimberMotor = 5; 
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
   * Config parameters for tuning the driver interface.
   */
  public static class Driver {

    /**
     * The joystick port for the driver's controller.
     */
    public static final int pilotPort = 1;
  }

  public static class Vision {
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final boolean kExtraVisionDebugInfo = true;

    //front-to-back-disp = ~8.568 inches 
    //left-to-right-disp = 0 inches
    //top-to-bottom disp = 17.82426 inches
    //public static final Rotation3d kCameraRotation = new Rotation3d(Math.PI, Units.degreesToRadians(-25), Math.PI);
    public static final Rotation3d kLimelightRotation = new Rotation3d(Units.degreesToRadians(0), 0, Units.degreesToRadians(0));
    public static final Translation3d kCameraOffset = new Translation3d(Units.inchesToMeters(3.213) - 0.08,Units.inchesToMeters(-0.495) + 0.02,Units.inchesToMeters(17.396));
    //public static final Transform3d kCameraTransfrom = new Transform3d(kCameraOffset, kCameraRotation);
    public static final Transform3d kLimelightTransform = new Transform3d(kCameraOffset, kLimelightRotation);

    public static final double closeAlignDistance = 0.75;
  }

  public static class CoralIntake {
    public static final double release = -0.3;
    public static final double intake = 0.5;
    public static final double neutral = 0.04;
    public static final double wiggle = 0.1;

    public static class Setpoints {
      public static final double pose1 = 0.08;
      public static final double pose2 = 0.16;
      public static final double pose3 = 0.24;
      public static final double pose4 = 0.33;
    }
  }

  public static class Elevator {
    public static final double kPSparkMax = 0.0135;
    public static final double kISparkMax = 0.0;
    public static final double kDSparkMax = 0;
    public static final double kGearRatio = 9.0;
    public static final double kMultiplier = 2.0;
    public static final double kConversionFactorRotationstoInches = Math.PI * 1.751;

    public static final double kFeedforwardG = 0.05;
    public static final double kFeedforwardV = 0.000;

    public static class Command {
      public static final double movementUpGain = 6; //9.0;
      public static final double movementDownGain = 5; //6.0;
      public static final double upMinSpeed = 5.75;
      public static final double downMinSpeed = 5;
      public static final double upMaxSpeed = 100;
      public static final double downMaxSpeed = 100;
      public static final double setPointMargin = 0.5;
      // public static final double simPositionFactor = 1; 
      public static final double elevatorUpRampUpTime = 0.5; //0.2;
      public static final double elevatorUpRampDownTime = 0.001; //0.01;
      public static final double elevatorDownRampUpTime = 0.1; //0.25;
      public static final double elevatorDownRampDownTime = 0.001; //0.01;
    }

    public static class Setpoints {
      public static final double L1 = 1;
      public static final double L2 = 2;
      public static final double L3 = 18.5;
      public static final double L4 = 50;
      public static final double Source = 1;
      public static final double Current = 1;
    }
}

  public static class Wrist {
    public static class Setpoints{
      public static final double L1 = 39 / 360.; // same as L4 for now
      public static final double L2 = 17 /360.;
      public static final double L3 = 20 / 360.;
      public static final double L4 = 39 / 360.;
      public static final double Source = -16 / 360.; //placehold
      public static final double Current = 0.23 * 360.;
      public static final double Drive = -64 / 360.;
    }

    public static final double WristKS = 0;
    public static final double WristKG = 0.51 / 12;
    public static final double WristKV = 0;

    public static final double WristKP = 3.2;//2.7079; //keep testing
    public static final double WristKI = 0;
    public static final double WristKD = 0.005;//0.69018;

    public static final double maxVelo = 3;
    public static final double maxAccel = 15;

    public static final double WristEncoderOffset = -0.75;

  }

  public static class Climber {
    public static final double upSpeed = 1;
    public static final double downSpeed = -1;
  }

  public static class AlgaeIntake {
    public static final double upPos = 90; // check
    public static final double downPos = 1; // probaby

    public static final double maxVelo = 1;
    public static final double maxAccel = 3;

    public static final double intakeSpeed = 0.1;
    public static final double expelSpeed = -0.1;
  }
}