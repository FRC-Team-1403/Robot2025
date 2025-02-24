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
    public static final int powerDistributionID = 60;
  
    public static final int candleID = kTBD;

    public static final int algaeIntakeMotorID = kTBD;
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
    public static final Rotation3d kCameraRotation = new Rotation3d(Math.PI, Units.degreesToRadians(-25), Math.PI);
    public static final Rotation3d kLimelightRotation = new Rotation3d(0, Units.degreesToRadians(-25), Math.PI);
    public static final Translation3d kCameraOffset = new Translation3d(Units.inchesToMeters(-8.568),0,Units.inchesToMeters(17.82426-48));
    public static final Transform3d kCameraTransfrom = new Transform3d(kCameraOffset, kCameraRotation);
    public static final Transform3d kLimelightTransform = new Transform3d(kCameraOffset, kLimelightRotation);
  }

}