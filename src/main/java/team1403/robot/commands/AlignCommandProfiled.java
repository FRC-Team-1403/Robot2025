package team1403.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.lib.util.CougarUtil;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;

public class AlignCommandProfiled extends Command {
    private final TrapezoidProfile.Constraints m_contraints = new TrapezoidProfile.Constraints(TunerConstants.kMaxSpeed, 12);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(7, 0, 0, m_contraints);
    private final ProfiledPIDController m_rotationController = 
        new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(3*Math.PI, 4*Math.PI));
    private final SwerveSubsystem m_swerve;
    private final Pose2d m_target;
    private final ChassisSpeeds speeds = new ChassisSpeeds();

    public AlignCommandProfiled(SwerveSubsystem swerve, Pose2d target) {
        m_swerve = swerve;
        m_target = target;

        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_controller.reset(CougarUtil.getDistance(m_target, m_swerve.getPose()), CougarUtil.norm(m_swerve.getState().Speeds));
        m_rotationController.reset(m_swerve.getRotation().getRadians(), m_swerve.getState().Speeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        Translation2d direction = m_target.getTranslation().minus(m_swerve.getPose().getTranslation()); //final-initial
        if(direction.getNorm() > 0.0001) {
            double unit_x = -direction.getX() / direction.getNorm(); //flip direction of the vector, should be going from initial to final
            double unit_y = -direction.getY() / direction.getNorm();
            double speed = m_controller.calculate(direction.getNorm(), 0);
            double rotSpeed = m_rotationController.calculate(m_swerve.getRotation().getRadians(), m_target.getRotation().getRadians());
            speeds.vxMetersPerSecond = unit_x * speed;
            speeds.vyMetersPerSecond = unit_y * speed;
            speeds.omegaRadiansPerSecond = rotSpeed;
            m_swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_swerve.getRotation()));
        } else {
            m_swerve.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return CougarUtil.getDistance(m_target, m_swerve.getPose()) < 0.01 &&
            CougarUtil.dot(m_swerve.getRotation(), m_target.getRotation()) >= Math.cos(Units.degreesToRadians(0.5));
    }
}
