package team1403.robot.commands;

import java.lang.constant.Constable;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.lib.util.CougarUtil;
import team1403.robot.Constants;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;

public class AlignCommand extends Command {

    private SwerveSubsystem m_swerve;
    private Pose2d m_target;
    private PPHolonomicDriveController m_driveController;
    private PathPlannerTrajectoryState m_state;

    private static final double kTreshM = 0.01;

    public AlignCommand(SwerveSubsystem swerve, Pose2d target) {
        m_swerve = swerve;
        m_target = target;

        m_driveController = 
        new PPHolonomicDriveController(
            TunerConstants.kTranslationPID,
            TunerConstants.kRotationPID,
            Constants.kLoopTime);
        m_state = new PathPlannerTrajectoryState();

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_state.pose = m_target;
    }
    
    @Override
    public void execute() {
        ChassisSpeeds speeds = m_driveController.calculateRobotRelativeSpeeds(m_swerve.getPose(), m_state);
        m_swerve.drive(speeds);
        if(CougarUtil.getDistance(m_target, m_swerve.getPose()) < Constants.Vision.closeAlignDistance)
            Blackbox.setCloseAlign(true);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return CougarUtil.getDistance(m_swerve.getPose(), m_target) <= kTreshM &&
                CougarUtil.dot(m_swerve.getRotation(), m_target.getRotation()) >= Math.cos(0.01);
    }

}
