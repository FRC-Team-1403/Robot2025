package team1403.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Meters;

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
  public static final ShuffleboardTab kDriverTab = Shuffleboard.getTab("Driver");
  public static final ShuffleboardTab kDebugTab = Shuffleboard.getTab("Debug");
  //controls if the debug tab is used on shuffleboard
  public static final boolean DEBUG_MODE = true;
 
  /**
   * Ports on the RoboRIO.
   */
  public static class RioPorts {
    
  }

  public static class CanBus {
    
    public static final int powerDistributionID = 42;

    public static final int leftElevatorMotorID = 10; //3;
    public static final int rightElevatorMotorID = 11; //2;
  
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
  }

  public static class Elevator {
      public static final double kPSparkMax = 0.0135;
      public static final double kISparkMax = 0.0;
      public static final double kDSparkMax = 0;

      public static final double kGearRatio = 9.0;
      public static final double kMultiplier = 2.0;
      public static final double kConversionFactorRotationstoInches = Math.PI * 1.751;

      public static final double first = 10;
      public static final double second = 30;
      public static final double third = 50;
      public static final double down = 1;
      public static final double kFeedforwardG = 0.02;
      public static final double kFeedforwardV = 0.001;

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
        public static final double elevatorDownRampUpTime = 0.25; //0.25;
        public static final double elevatorDownRampDownTime = 0.001; //0.01;
      }
  }
}