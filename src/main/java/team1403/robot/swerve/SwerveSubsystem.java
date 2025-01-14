package team1403.robot.swerve;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;


import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team1403.lib.util.CougarUtil;
import team1403.robot.Constants;
import team1403.robot.Robot;
import team1403.robot.Constants.CanBus;
import team1403.robot.Constants.Swerve;
import team1403.robot.swerve.ISwerveModule.DriveControlType;
import team1403.robot.swerve.ISwerveModule.SteerControlType;
import team1403.robot.vision.AprilTagCamera;
import team1403.robot.vision.ITagCamera;
import team1403.robot.vision.VisionSimUtil;

import static edu.wpi.first.units.Units.Volts;

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

  private final ArrayList<ITagCamera> m_cameras = new ArrayList<>();
  private boolean m_disableVision = false;
  private boolean m_rotDriftCorrect = true;
  private final SwerveHeadingCorrector m_headingCorrector = new SwerveHeadingCorrector();
  private SimDouble m_gryoHeadingSim;
  private SimDouble m_gyroRateSim;
  private SysIdRoutine m_sysIdRoutine;
  private SysIdRoutine m_sysIDAngle;

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
        this::getPose, // Robot pose supplier
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
      Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
      m_field.getObject("traj").setPoses(activePath);
    });
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
      Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    });

    // addDevice(m_navx2.getName(), m_navx2);
    if (m_navx2.isConnected())
      while (m_navx2.isCalibrating());

    zeroGyroscope();

    //initialize the arrays
    getModuleStates();
    getModulePositions();

    m_odometer = new SyncSwerveDrivePoseEstimator(CougarUtil.getInitialRobotPose(), () -> getGyroscopeRotation(), () -> getModulePositions());

    VisionSimUtil.initVisionSim();

    m_cameras.add(new AprilTagCamera("Unknown_Camera", () -> Swerve.kCameraTransfrom, this::getPose));

    m_odometeryNotifier = new Notifier(m_odometer::update);
    m_odometeryNotifier.setName("SwerveOdoNotifer");
    m_odometeryNotifier.startPeriodic(Units.millisecondsToSeconds(Constants.Swerve.kModuleUpdateRateMs));

    m_sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, null, null, 
      (state) -> Logger.recordOutput("SysIDSwerveLinear", state.toString())),
    new SysIdRoutine.Mechanism((voltage) -> {
      for(ISwerveModule m : m_modules) {
        m.set(DriveControlType.Voltage, voltage.in(Volts), SteerControlType.Angle, 0);
      }
    }, null, this));
    m_sysIDAngle = new SysIdRoutine(new SysIdRoutine.Config(null, null, null, 
      (state) -> Logger.recordOutput("SysIDSwerveSteer", state.toString())), 
      new SysIdRoutine.Mechanism((voltage) -> {
        for(ISwerveModule m : m_modules) {
          m.set(DriveControlType.Voltage, 0, SteerControlType.Voltage, voltage.in(Volts));
        }
      }, null, this));

    SmartDashboard.putData("Gyro", m_navx2);
    SmartDashboard.putData("Field", m_field);
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
      resetOdometry(CougarUtil.createPose2d(getPose(), Rotation2d.kZero));
    else
      resetOdometry(CougarUtil.createPose2d(getPose(), Rotation2d.k180deg));
    m_headingCorrector.resetHeadingSetpoint();
  }

  /**
   * Return the position of the drivetrain.
   *
   * @return the position of the drivetrain in Pose3d
   */
  public Pose2d getPose() {
    return m_odometer.getPose();
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
    m_odometer.resetPosition(pose);
  }

  /**
   * Gets the heading of the gyroscope.
   *
   * @return a Rotation3d object that contains the gyroscope's heading
   */
  private Rotation2d getGyroscopeRotation() {
    return m_navx2.getRotation2d();
  }

  
  /**
   * Accounts for the drift caused by the first order kinematics
   * while doing both translational and rotational movement.
   * 
   * <p>
   * Looks forward one control loop to figure out where the robot
   * should be given the chassisspeed and backs out a twist command from that.
   * 
   * @param chassisSpeeds the given chassisspeeds
   * @return the corrected chassisspeeds
   */
  private ChassisSpeeds translationalDriftCorrection(ChassisSpeeds chassisSpeeds) {
    double dtheta = Units.degreesToRadians(m_navx2.getRate()) * Constants.Swerve.kAngVelCoeff;
    Logger.recordOutput("test", dtheta);
    if(Math.abs(dtheta) > 0.001 && Math.abs(dtheta) < 5 && Robot.isReal()) {
      Rotation2d rot = getRotation();
      chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, rot);
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, rot.plus(new Rotation2d(dtheta)));
    }

    return chassisSpeeds;
  }

  /**
   * Gets the heading of the robot.
   *
   * @return a Rotation2d object that contains the robot's heading
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Sets the target chassis speeds
   * @param chassisSpeeds
   */
  public void drive(ChassisSpeeds chassisSpeeds, boolean discretize) {
    m_chassisSpeeds = translationalDriftCorrection(chassisSpeeds);
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
      m_modules[i].set(DriveControlType.Velocity, states[i].speedMetersPerSecond,
          SteerControlType.Angle, MathUtil.angleModulus(states[i].angle.getRadians()));
    }

    Logger.recordOutput("SwerveStates/Target", states);
  }


  public SwerveModuleState[] getModuleStates() {
    for(int i = 0; i < m_modules.length; i++) {
      m_currentStates[i] = m_modules[i].getState();
    }
    return m_currentStates;
  }

  public ChassisSpeeds getTargetChassisSpeed() {
    return m_chassisSpeeds;
  }

  public ChassisSpeeds getCurrentChassisSpeed() {
    ChassisSpeeds ret =  Swerve.kDriveKinematics.toChassisSpeeds(getModuleStates());
    Logger.recordOutput("SwerveStates/Current Chassis Speeds", ret);
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
    ChassisSpeeds corrected = m_headingCorrector.update(speeds, getCurrentChassisSpeed(), getRotation(), m_navx2.getRate());
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

    builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
  }

  public Command getSysIDQ(SysIdRoutine.Direction dir) {
    return m_sysIdRoutine.quasistatic(dir);
  }

  public Command getSysIDD(SysIdRoutine.Direction dir) {
    return m_sysIdRoutine.dynamic(dir);
  }

  public Command getSysIDSteerQ(SysIdRoutine.Direction dir) {
    return m_sysIDAngle.quasistatic(dir);
  }

  public Command getSysIDSteerD(SysIdRoutine.Direction dir) {
    return m_sysIDAngle.dynamic(dir);
  }

  private Pose2d[] getModulePoses() {
    Pose2d[] ret = new Pose2d[m_modules.length];
    Pose2d cur = getPose();
    
    for(int i = 0; i < ret.length; i++) {
      ret[i] = cur.transformBy(
        new Transform2d(Constants.Swerve.kModulePositions[i], 
        m_currentStates[i].angle));
    }

    return ret;
  }

  private void updateTargetModuleStates(boolean discretize) {
    ChassisSpeeds corrected = rotationalDriftCorrection(m_chassisSpeeds);

    Logger.recordOutput("SwerveStates/Corrected Target Chassis Speeds", corrected);

    setModuleStates(Swerve.kDriveKinematics.toSwerveModuleStates(corrected), discretize);
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Odometry/Cycles", m_odometer.resetUpdateCount());

    if(!m_disableVision)
    {
      for(ITagCamera cam : m_cameras)
      {
        if (cam.checkVisionResult()) {
          Pose3d pose = cam.getPose();
          if (pose != null) {
            m_odometer.addVisionMeasurement(pose.toPose2d(), cam.getTimestamp(), cam.getEstStdv());
          }
        }
      }
    }
    // SmartDashboard.putNumber("Speed", m_speedLimiter);

    m_field.setRobotPose(getPose());
    if (Constants.DEBUG_MODE) m_field.getObject("xModules").setPoses(getModulePoses());
    // Logging Output

    Logger.recordOutput("SwerveStates/Target Chassis Speeds", m_chassisSpeeds);

    //wip: slip detection based on orbit's swerve presentation
    /*
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
    

    Logger.recordOutput("SwerveStates/Ratio", Math.abs(min) < 0.01 ? 1 : max/min);
    Logger.recordOutput("SwerveStates/PureTranslation", tState); */
    Logger.recordOutput("SwerveStates/Measured", m_currentStates);
    Logger.recordOutput("Odometry/Robot", getPose());
    Logger.recordOutput("Odometry/Rotation3d", m_navx2.getRotation3d());
  }
}
