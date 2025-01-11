package team1403.robot.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.lib.util.CircularSlewRateLimiter;
import team1403.lib.util.CougarUtil;
import team1403.robot.Constants;
import team1403.robot.Robot;
import team1403.robot.Constants.Swerve;
import team1403.robot.subsystems.Blackbox;

/**
 * The default command for the swerve drivetrain subsystem.
 */
public class DefaultSwerveCommand extends Command {
  private final SwerveSubsystem m_drivetrainSubsystem;

  private final DoubleSupplier m_verticalTranslationSupplier;
  private final DoubleSupplier m_horizontalTranslationSupplier;
  private final DoubleSupplier m_rotationSupplier;
  private final BooleanSupplier m_fieldRelativeSupplier;
  private final BooleanSupplier m_xModeSupplier;
  private final BooleanSupplier m_aimbotSupplier;
  private final DoubleSupplier m_speedSupplier;
  private final Supplier<Translation2d> m_targetPosSupplier;
  private final DoubleSupplier m_snipingMode;
  private final BooleanSupplier m_ampSupplier;
  private final BooleanSupplier m_alignSupplier;
  private boolean m_isFieldRelative;
  
  private SlewRateLimiter m_translationLimiter;
  private SlewRateLimiter m_rotationRateLimiter;
  private CircularSlewRateLimiter m_directionSlewRate;
  private static final double kDirectionSlewRateLimit = 22;

  private ProfiledPIDController m_controller;
  private TrapezoidProfile.State m_state = new TrapezoidProfile.State();
  private PPHolonomicDriveController m_driveController = new PPHolonomicDriveController(
    Constants.PathPlanner.kTranslationPID,
    Constants.PathPlanner.kRotationPID,
    Constants.kLoopTime
  );
  private PathPlannerTrajectoryState m_driveState = new PathPlannerTrajectoryState();

  private double m_speedLimiter = 0.2;

  /**
   * Creates the swerve command.
   * \
   * 
   * @param drivetrain                    the instance of the
   *                                      {@link SwerveSubsystem}
   * @param horizontalTranslationSupplier
   *                                      supplies the horizontal speed of the
   *                                      drivetrain
   * @param verticalTranslationSupplier
   *                                      supplies the the vertical speed of the
   *                                      drivetrain
   * @param rotationSupplier              supplies the rotational speed of the
   *                                      drivetrain
   * @param fieldRelativeSupplier         supplies the
   *                                      boolean value to enable field relative
   *                                      mode
   */
  public DefaultSwerveCommand(SwerveSubsystem drivetrain,
      DoubleSupplier horizontalTranslationSupplier,
      DoubleSupplier verticalTranslationSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldRelativeSupplier,
      BooleanSupplier xModeSupplier,
      BooleanSupplier aimbotSupplier,
      BooleanSupplier ampSupplier,
      BooleanSupplier alignmentSupplier,
      Supplier<Translation2d> targetSupplier,
      DoubleSupplier speedSupplier,
      DoubleSupplier snipingMode) {
    this.m_drivetrainSubsystem = drivetrain;
    this.m_verticalTranslationSupplier = verticalTranslationSupplier;
    this.m_horizontalTranslationSupplier = horizontalTranslationSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.m_fieldRelativeSupplier = fieldRelativeSupplier;
    this.m_speedSupplier = speedSupplier;
    this.m_xModeSupplier = xModeSupplier;
    this.m_aimbotSupplier = aimbotSupplier;
    this.m_targetPosSupplier = targetSupplier;
    this.m_alignSupplier = alignmentSupplier;
    m_ampSupplier = ampSupplier;
    m_snipingMode = snipingMode;
    m_isFieldRelative = true;

    m_translationLimiter = new SlewRateLimiter(2, -2, 0);
    m_rotationRateLimiter = new SlewRateLimiter(4, -4, 0);
    m_directionSlewRate = new CircularSlewRateLimiter(kDirectionSlewRateLimit);

    m_controller = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(Swerve.kMaxAngularSpeed, 80));
    m_controller.enableContinuousInput(-Math.PI, Math.PI);

    Constants.kDriverTab.addBoolean("isFieldRelative", () -> m_isFieldRelative);
    if(Constants.DEBUG_MODE) {
      Constants.kDebugTab.addBoolean("Aimbot", m_aimbotSupplier);
      Constants.kDebugTab.add("Swerve Rotation PID", m_controller);
    }

    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    ChassisSpeeds speeds = m_drivetrainSubsystem.getCurrentChassisSpeed();
    m_controller.reset(MathUtil.angleModulus(m_drivetrainSubsystem.getRotation().getZ()), speeds.omegaRadiansPerSecond);
    m_driveController.reset(m_drivetrainSubsystem.getPose2D(), speeds);
  }

  @Override
  public void execute() {
    m_speedLimiter = 0.3 * (1.0 - m_snipingMode.getAsDouble() * 0.7) + (m_speedSupplier.getAsDouble() * 0.7);
  
    if (DriverStation.isAutonomousEnabled()) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds(), false);
      return;
    }

    if (m_fieldRelativeSupplier.getAsBoolean()) {
      m_isFieldRelative = !m_isFieldRelative;
    }

    if (m_xModeSupplier.getAsBoolean()) {
      m_drivetrainSubsystem.xMode();
      return;
    }

    ChassisSpeeds chassisSpeeds;
    ChassisSpeeds currentSpeeds = m_drivetrainSubsystem.getCurrentChassisSpeed();
    double horizontal = m_horizontalTranslationSupplier.getAsDouble();
    double vertical = m_verticalTranslationSupplier.getAsDouble();
    double vel_hypot = Math.hypot(horizontal, vertical);

    if(CougarUtil.getAlliance() == Alliance.Red && m_isFieldRelative) {
      horizontal *= -1;
      vertical *= -1;
    }

    {
      //normalize using polar coordinates
      double velocity = MathUtil.clamp(vel_hypot, 0, 1);
      velocity = MathUtil.applyDeadband(velocity, 0.05);
      double angle = Math.atan2(vertical, horizontal);

      velocity *= m_speedLimiter;
      velocity = m_translationLimiter.calculate(velocity) * Constants.Swerve.kMaxSpeed;

      if(vel_hypot < 0.01) {
        angle = m_directionSlewRate.lastValue();
      }
      else if(velocity < 0.01) {
        m_directionSlewRate.setLimits(500);
        angle = m_directionSlewRate.calculate(angle);
      }
      else {
        m_directionSlewRate.setLimits(kDirectionSlewRateLimit / velocity);
        angle = m_directionSlewRate.calculate(angle);
      }

      horizontal = velocity * Math.cos(angle);
      vertical = velocity * Math.sin(angle);
    }
    double ang_deadband = MathUtil.applyDeadband(m_rotationSupplier.getAsDouble(), 0.05);
    double angular = m_rotationRateLimiter.calculate(squareNum(ang_deadband) * m_speedLimiter) * Swerve.kMaxAngularSpeed;

    Pose2d curPose = m_drivetrainSubsystem.getPose2D();
    Rotation2d curRotation = curPose.getRotation();
    double given_current_angle = MathUtil.angleModulus(curRotation.getRadians());
    double given_target_angle = Math.atan2(m_targetPosSupplier.get().getY() - curPose.getY(), m_targetPosSupplier.get().getX() - curPose.getX());
    given_target_angle = MathUtil.angleModulus(given_target_angle + Math.PI);
    // double given_target_angle = Units.radiansToDegrees(Math.atan2(m_drivetrainSubsystem.getPose().getY() - m_ysupplier.getAsDouble(), m_drivetrainSubsystem.getPose().getX() - m_xsupplier.getAsDouble()));

    Logger.recordOutput("SwerveDC/Target Angle", given_target_angle);
    
    if(m_aimbotSupplier.getAsBoolean())
    {
      angular = m_controller.calculate(given_current_angle, given_target_angle);
      Logger.recordOutput("SwerveDC/Aimbot Angular", angular);
    } else {
      m_state.position = given_current_angle;
      m_state.velocity = currentSpeeds.omegaRadiansPerSecond;
      m_controller.reset(m_state);
    }
    
    if((m_ampSupplier.getAsBoolean() || m_alignSupplier.getAsBoolean()) && Blackbox.isValidTargetPosition() && vel_hypot < 0.2) {
      m_driveState.pose = Blackbox.getTargetPosition();
      chassisSpeeds = m_driveController.calculateRobotRelativeSpeeds(curPose, m_driveState);
      double output_speed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
      if(output_speed < 0.02) {
        chassisSpeeds.vxMetersPerSecond = 0;
        chassisSpeeds.vyMetersPerSecond = 0;
      }
      if(Math.abs(chassisSpeeds.omegaRadiansPerSecond) < 0.02) {
        chassisSpeeds.omegaRadiansPerSecond = 0;
      }
    } else {
      chassisSpeeds = new ChassisSpeeds(vertical, horizontal, angular);
      if (m_isFieldRelative) {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, curRotation);
      }
      m_driveController.reset(curPose, currentSpeeds);
      //chassisSpeeds = translationalDriftCorrection(chassisSpeeds);
    }

    m_drivetrainSubsystem.drive(chassisSpeeds, true);
  }

  private static double squareNum(double num) {
    return Math.signum(num) * Math.pow(num, 2);
  }
}
