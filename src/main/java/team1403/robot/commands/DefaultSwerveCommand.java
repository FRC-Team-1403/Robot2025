package team1403.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.lib.util.CougarUtil;
import team1403.robot.Constants;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;

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
  private final DoubleSupplier m_speedSupplier;
  private final DoubleSupplier m_snipingMode;
  private boolean m_isFieldRelative;
  
  private SlewRateLimiter m_rotationRateLimiter;
  private double prev_horizontal = 0;
  private double prev_vertical = 0;
  private static final double kMaxVelocityChange = 13 * Constants.kLoopTime;

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
      DoubleSupplier speedSupplier,
      DoubleSupplier snipingMode) {
    this.m_drivetrainSubsystem = drivetrain;
    this.m_verticalTranslationSupplier = verticalTranslationSupplier;
    this.m_horizontalTranslationSupplier = horizontalTranslationSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.m_fieldRelativeSupplier = fieldRelativeSupplier;
    this.m_speedSupplier = speedSupplier;
    this.m_xModeSupplier = xModeSupplier;
    m_snipingMode = snipingMode;
    m_isFieldRelative = true;
    m_rotationRateLimiter = new SlewRateLimiter(3, -3, 0);

    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    prev_horizontal = prev_vertical = 0;
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("isFieldRelative", m_isFieldRelative);
    //if (Constants.DEBUG_MODE) SmartDashboard.putBoolean("Aimbot", m_aimbotSupplier.getAsBoolean());

    m_speedLimiter = 0.3 * (1.0 - m_snipingMode.getAsDouble() * 0.7) + (m_speedSupplier.getAsDouble() * 0.7);
  
    if (DriverStation.isAutonomousEnabled()) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds());
      return;
    }

    if (m_fieldRelativeSupplier.getAsBoolean()) {
      m_isFieldRelative = !m_isFieldRelative;
    }

    if (m_xModeSupplier.getAsBoolean()) {
      m_drivetrainSubsystem.setControl(new SwerveRequest.SwerveDriveBrake());
      return;
    }

    ChassisSpeeds chassisSpeeds;
    double horizontal = m_horizontalTranslationSupplier.getAsDouble();
    double vertical = m_verticalTranslationSupplier.getAsDouble();
    double vel_hypot = Math.hypot(horizontal, vertical);

    if(CougarUtil.getAlliance() == Alliance.Red && m_isFieldRelative) {
      horizontal *= -1;
      vertical *= -1;
    }

    if(vel_hypot > 0)
    {
      //normalize using vector scaling
      double velocity = MathUtil.clamp(vel_hypot, 0, 1);
      velocity = MathUtil.applyDeadband(velocity, 0.05);
      
      //scale unit vector by speed limiter and convert to speed
      horizontal *= velocity / vel_hypot * TunerConstants.kMaxSpeed * m_speedLimiter;
      vertical *= velocity / vel_hypot * TunerConstants.kMaxSpeed * m_speedLimiter;
    }
    double ang_deadband = MathUtil.applyDeadband(m_rotationSupplier.getAsDouble(), 0.05);
    double angular = m_rotationRateLimiter.calculate(squareNum(ang_deadband) * m_speedLimiter) * TunerConstants.kMaxAngularRate;
    // double given_target_angle = Units.radiansToDegrees(Math.atan2(m_drivetrainSubsystem.getPose().getY() - m_ysupplier.getAsDouble(), m_drivetrainSubsystem.getPose().getX() - m_xsupplier.getAsDouble()));
    
    //limit change in translation of the overall robot, based on orbit's slideshow
    {
      double dx = horizontal - prev_horizontal;
      double dy = vertical - prev_vertical;
      double dmag = Math.hypot(dx, dy);

      if (dmag > 0) {
        double scale = MathUtil.clamp(dmag, 0, kMaxVelocityChange) / dmag;
        dx *= scale;
        dy *= scale;
        horizontal = prev_horizontal + dx;
        vertical = prev_vertical + dy;
      }

      prev_horizontal = horizontal;
      prev_vertical = vertical;
    }

    {
      if (m_isFieldRelative) {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vertical, horizontal, angular, m_drivetrainSubsystem.getShallowRotation());
      } else {
        chassisSpeeds = new ChassisSpeeds(vertical, horizontal, angular);
      }
      //chassisSpeeds = translationalDriftCorrection(chassisSpeeds);
    }

    m_drivetrainSubsystem.drive(chassisSpeeds);
  }

  private static double squareNum(double num) {
    return Math.signum(num) * Math.pow(num, 2);
  }

  // @Override
  // private void periodic() {
  //   Pose2d curPose = m_drivetrainSubsystem.getPose();
  //   Rotation2d curRotation = curPose.getRotation();
  //   double given_current_angle = MathUtil.angleModulus(curRotation.getRadians());
  //   double given_target_angle = Math.atan2(m_targetPosSupplier.get().getY() - curPose.getY(), m_targetPosSupplier.get().getX() - curPose.getX());
  //   given_target_angle = MathUtil.angleModulus(given_target_angle + Math.PI);
  // }
}
