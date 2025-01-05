package team1403.robot.swerve;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import dev.doglog.DogLog;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.util.CougarUtil;
import team1403.robot.Constants;
import team1403.robot.Robot;
import team1403.robot.Constants.CanBus;
import team1403.robot.Constants.Swerve;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Meters;

/**
 * The drivetrain of the robot. Consists of for swerve modules and the
 * gyroscope.
 */
public class SwerveSubsystem extends SubsystemBase {
  private final AHRS m_navx2;
  private final ISwerveModule[] m_modules;
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  private final SwerveModuleState[] m_currentStates = new SwerveModuleState[4];
  private final SwerveModulePosition[] m_currentPositions = new SwerveModulePosition[4];
  private final SyncSwerveDrivePoseEstimator m_odometer;
  private final Field2d m_field = new Field2d();

  private final ArrayList<AprilTagCamera> m_cameras = new ArrayList<>();
  private boolean m_disableVision = false;
  private boolean m_rotDriftCorrect = true;
  private final SwerveHeadingCorrector m_headingCorrector = new SwerveHeadingCorrector();
  private SimDouble m_gryoHeadingSim;
  private SimDouble m_gyroRateSim;

  private static final SwerveModuleState[] m_xModeState = {
    // Front Left
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    // Front Right
    new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
    // Back left
    new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
    // Back Right
    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
  };

  private final Notifier m_odometeryNotifier;

  //patched warmup command so it's not slow af
  public static Command swerveWarmupCommand() {
    return new PathfindingCommand(
            new Pose2d(15.0, 4.0, Rotation2d.k180deg),
            new PathConstraints(4, 3, 4, 4),
            () -> new Pose2d(1.5, 4, Rotation2d.kZero),
            ChassisSpeeds::new,
            (speeds, feedforwards) -> {},
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            CougarUtil.loadRobotConfig())
        .andThen(Commands.print("[PathPlanner] PathfindingCommand finished warmup"))
        .ignoringDisable(true);
  }

  /**
   * Creates a new {@link SwerveSubsystem}.
   * Instantiates the 4 {@link SwerveModule}s,
   * the {@link SwerveDriveOdometry}, and the {@link NavxAhrs}.
   * Also sets drivetrain ramp rate,
   * and idle mode to default values.
   *
   * @param parameters the {@link CougarLibInjectedParameters}
   *                   used to construct this subsystem
   */
  public SwerveSubsystem() {
    // increase update rate because of async odometery
    m_navx2 = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k100Hz);
    if(Robot.isReal()) {
      m_modules = new ISwerveModule[] {
          new SwerveModule("Front Left Module",
              CanBus.frontLeftDriveID, CanBus.frontLeftSteerID,
              CanBus.frontLeftEncoderID, Swerve.frontLeftEncoderOffset),
          new SwerveModule("Front Right Module",
              CanBus.frontRightDriveID, CanBus.frontRightSteerID,
              CanBus.frontRightEncoderID, Swerve.frontRightEncoderOffset),
          new SwerveModule("Back Left Module",
              CanBus.backLeftDriveID, CanBus.backLeftSteerID,
              CanBus.backLeftEncoderID, Swerve.backLeftEncoderOffset),
          new SwerveModule("Back Right Module",
              CanBus.backRightDriveID, CanBus.backRightSteerID,
              CanBus.backRightEncoderID, Swerve.backRightEncoderOffset),
      };
    } else {
      m_modules = new ISwerveModule[] {
        new SimSwerveModule("Front Left Module"),
        new SimSwerveModule("Front Right Module"),
        new SimSwerveModule("Back Left Module"),
        new SimSwerveModule("Back Right Module")
      };

      int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]");
      m_gryoHeadingSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
      m_gyroRateSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Rate"));
    }

    AutoBuilder.configure(
        this::getPose2D, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getCurrentChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (ChassisSpeeds s) -> drive(s, true), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new PPHolonomicDriveController(
          Constants.PathPlanner.kTranslationPID, 
          Constants.PathPlanner.kRotationPID, 
          Constants.kLoopTime),
        CougarUtil.loadRobotConfig(),
        CougarUtil::shouldMirrorPath,
        this // Reference to this subsystem to set requirements
    );
    Pathfinding.setPathfinder(new LocalADStar());
    //replace when pathplanner warmup command gets fixed (update: it never did lmao)
    swerveWarmupCommand().schedule();
    PathPlannerLogging.setLogActivePathCallback((activePath) -> {
      DogLog.log("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
      m_field.getObject("traj").setPoses(activePath);
    });
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
      DogLog.log("Odometry/TrajectorySetpoint", targetPose);
    });

    // addDevice(m_navx2.getName(), m_navx2);
    if (m_navx2.isConnected())
      while (m_navx2.isCalibrating());

    zeroGyroscope();

    m_odometer = new SyncSwerveDrivePoseEstimator(CougarUtil.getInitialRobotPose(), () -> getGyroscopeRotation(), () -> getModulePositions());

    VisionSimUtil.initVisionSim();

    m_cameras.add(new AprilTagCamera("Unknown_Camera", () -> Swerve.kCameraTransfrom, this::getPose));

    m_odometeryNotifier = new Notifier(m_odometer::update);
    m_odometeryNotifier.setName("SwerveOdoNotifer");
    m_odometeryNotifier.startPeriodic(Units.millisecondsToSeconds(Constants.Swerve.kModuleUpdateRateMs));

    Constants.kDriverTab.add("Gyro", m_navx2);
    Constants.kDriverTab.add("Field", m_field);
  }

  public void setDisableVision(boolean disable) {
    m_disableVision = disable;
  }

  /**
   * Gets the 4 swerve module positions.
   *
   * @return an array of swerve module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    for(int i = 0; i < m_modules.length; i++) {
      m_currentPositions[i] = m_modules[i].getModulePosition();
    }
    return m_currentPositions;
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' directi=on.
   */
  private void zeroGyroscope() {
    // tracef("zeroGyroscope %f", getGyroscopeRotation());
    m_navx2.reset();
  }

  public void zeroHeading() {
    zeroGyroscope();
    if(CougarUtil.getAlliance() == Alliance.Blue)
      resetOdometry(CougarUtil.createPose3d(getPose(), new Rotation3d()));
    else
      resetOdometry(CougarUtil.createPose3d(getPose(), new Rotation3d(0, 0, Math.PI)));
    m_headingCorrector.resetHeadingSetpoint();
  }

  /**
   * Return the position of the drivetrain.
   *
   * @return the position of the drivetrain in Pose3d
   */
  public Pose3d getPose() {
    return m_odometer.getPose();
  }

  /**
   * Return the position of the drivetrain.
   *
   * @return the position of the drivetrain in Pose2d
   */
  public Pose2d getPose2D() {
    return getPose().toPose2d();
  }

  /**
   * Reset the position of the drivetrain odometry.
   */
  public void resetOdometry() {
    resetOdometry(getPose());
  }

  /**
   * Reset the position of the drivetrain odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetOdometry(new Pose3d(pose));
  }

    /**
   * Reset the position of the drivetrain odometry.
   */
  public void resetOdometry(Pose3d pose) {
    m_odometer.resetPosition(pose);
  }

  /**
   * Gets the heading of the gyroscope.
   *
   * @return a Rotation3d object that contains the gyroscope's heading
   */
  private Rotation3d getGyroscopeRotation() {
    //work around navx sim bug
    if(Robot.isSimulation()) return new Rotation3d(m_navx2.getRotation2d());

    return m_navx2.getRotation3d();
  }

  /**
   * Gets the heading of the robot.
   *
   * @return a Rotation2d object that contains the robot's heading
   */
  public Rotation3d getRotation() {
    return getPose().getRotation();
  }

  public double getRate() {
    return m_navx2.getRate();
  }

  /**
   * Sets the target chassis speeds
   * @param chassisSpeeds
   */
  public void drive(ChassisSpeeds chassisSpeeds, boolean discretize) {
    m_chassisSpeeds = chassisSpeeds;
    //update here to reduce latency
    updateTargetModuleStates(discretize);
  }

  /**
   * Stops the drivetrain.
   */
  public void stop() {
    m_chassisSpeeds = new ChassisSpeeds();
    updateTargetModuleStates(false);
  }

  /**
   * Sets the module speed and heading for all 4 modules.
   *
   * @param states an array of states for each module.
   */
  
  public void setModuleStates(SwerveModuleState[] states, boolean discretize) {
    SwerveModuleState[] currentStates = getModuleStates();

    //desaturate sandwich :)
    if(discretize) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.kMaxSpeed);
      ChassisSpeeds temp = Constants.Swerve.kDriveKinematics.toChassisSpeeds(states);
      temp = ChassisSpeeds.discretize(temp, Constants.kLoopTime);
      states = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(temp);
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.kMaxSpeed);

    for (int i = 0; i < m_modules.length; i++) {
      states[i].optimize(currentStates[i].angle);
      m_modules[i].set(states[i].speedMetersPerSecond,
          MathUtil.angleModulus(states[i].angle.getRadians()));
    }

    DogLog.log("SwerveStates/Target", states);
  }


  public SwerveModuleState[] getModuleStates() {
    for(int i = 0; i < m_modules.length; i++) {
      m_currentStates[i] = m_modules[i].getState();
    }
    DogLog.log("SwerveStates/Measured", m_currentStates);
    return m_currentStates;
  }

  public ChassisSpeeds getTargetChassisSpeed() {
    return m_chassisSpeeds;
  }

  public ChassisSpeeds getCurrentChassisSpeed() {
    ChassisSpeeds ret =  Swerve.kDriveKinematics.toChassisSpeeds(getModuleStates());
    DogLog.log("SwerveStates/Current Chassis Speeds", ret);
    return ret;
  }

  /**
   * Puts the drivetrain into xMode where all the wheel put towards the center of
   * the robot, 
   * making it harder for the robot to be pushed around.
   */
  public void xMode() {
    setModuleStates(m_xModeState, false);
    m_chassisSpeeds = new ChassisSpeeds();
  }

  private ChassisSpeeds rotationalDriftCorrection(ChassisSpeeds speeds) {
    ChassisSpeeds corrected = m_headingCorrector.update(speeds, getCurrentChassisSpeed(), getRotation().toRotation2d(), m_navx2.getRate());
    if (m_rotDriftCorrect && !DriverStation.isAutonomousEnabled())
    {
      return corrected;
    }

    return speeds;
  }

  @Override
  public void simulationPeriodic() {
    VisionSimUtil.update(getPose());
    double vel = getCurrentChassisSpeed().omegaRadiansPerSecond;
    m_gyroRateSim.set(Units.radiansToDegrees(-vel));
    m_gryoHeadingSim.set(m_gryoHeadingSim.get() - Units.radiansToDegrees(vel) * Constants.kLoopTime);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");

    builder.addDoubleProperty("Front Left Angle", () -> m_currentStates[0].angle.getRadians(), null);
    builder.addDoubleProperty("Front Left Velocity", () -> m_currentStates[0].speedMetersPerSecond, null);

    builder.addDoubleProperty("Front Right Angle", () -> m_currentStates[1].angle.getRadians(), null);
    builder.addDoubleProperty("Front Right Velocity", () -> m_currentStates[1].speedMetersPerSecond, null);

    builder.addDoubleProperty("Back Left Angle", () -> m_currentStates[2].angle.getRadians(), null);
    builder.addDoubleProperty("Back Left Velocity", () -> m_currentStates[2].speedMetersPerSecond, null);

    builder.addDoubleProperty("Back Right Angle", () -> m_currentStates[3].angle.getRadians(), null);
    builder.addDoubleProperty("Back Right Velocity", () -> m_currentStates[3].speedMetersPerSecond, null);

    builder.addDoubleProperty("Robot Angle", () -> getRotation().getZ(), null);
  }

  private Pose2d[] getModulePoses() {
    Pose2d[] ret = new Pose2d[m_modules.length];
    Pose2d cur = getPose2D();
    
    for(int i = 0; i < ret.length; i++) {
      ret[i] = cur.transformBy(
        new Transform2d(Constants.Swerve.kModulePositions[i], 
        m_currentStates[i].angle));
    }

    return ret;
  }

  private void updateTargetModuleStates(boolean discretize) {
    ChassisSpeeds corrected = rotationalDriftCorrection(m_chassisSpeeds);

    DogLog.log("SwerveStates/Corrected Target Chassis Speeds", corrected);

    setModuleStates(Swerve.kDriveKinematics.toSwerveModuleStates(corrected), discretize);
  }

  @Override
  public void periodic() {

    DogLog.log("Odometry/Cycles", m_odometer.resetUpdateCount());

    if(!m_disableVision)
    {
      for(AprilTagCamera cam : m_cameras)
      {
        if (cam.hasPose()) {
          Pose3d pose = cam.getPose();
          if (pose != null && cam.checkVisionResult()) {
            m_odometer.addVisionMeasurement(pose, cam.getTimestamp(), cam.getEstStdv());
          }
        }
      }
    }
    // SmartDashboard.putNumber("Speed", m_speedLimiter);

    m_field.setRobotPose(getPose2D());
    if (Constants.DEBUG_MODE) m_field.getObject("xModules").setPoses(getModulePoses());
    // Logging Output

    DogLog.log("SwerveStates/Target Chassis Speeds", m_chassisSpeeds);

    //wip: slip detection based on orbit's swerve presentation
    ChassisSpeeds temp = getCurrentChassisSpeed();
    temp.vxMetersPerSecond = 0;
    temp.vyMetersPerSecond = 0;
    SwerveModuleState[] rotStates = Swerve.kDriveKinematics.toSwerveModuleStates(temp);
    SwerveModuleState[] curStates = getModuleStates();
    SwerveModuleState[] tState = new SwerveModuleState[4];

    for(int i = 0; i < Constants.Swerve.kNumSwerveModules; i++) {
      SwerveModuleState rotState = rotStates[i];
      SwerveModuleState curState = curStates[i];
      Translation2d curT = new Translation2d(curState.speedMetersPerSecond, curState.angle);
      Translation2d rotT = new Translation2d(rotState.speedMetersPerSecond, rotState.angle);
      Translation2d diff = curT.minus(rotT);
      tState[i] = new SwerveModuleState(diff.getNorm(), diff.getAngle());
    }

    double min = Double.MAX_VALUE, max = 0;

    for(SwerveModuleState s : tState) {
      double speed = Math.abs(s.speedMetersPerSecond);
      min = Math.min(min, speed);
      max = Math.max(max, speed);
    }

    DogLog.log("SwerveStates/Ratio", Math.abs(min) < 0.01 ? 1 : max/min);
    DogLog.log("SwerveStates/PureTranslation", tState);
    DogLog.log("Odometry/Robot", getPose());
  }
}
