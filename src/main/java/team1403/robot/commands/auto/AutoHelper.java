package team1403.robot.commands.auto;

import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.lib.util.CougarUtil;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;

public class AutoHelper {
    
    //can't be a pathplanner auto since we should be able to run this from anywhere on the starting line
    public static Command getMoveAuto(SwerveSubsystem m_swerve) {
        return m_swerve.defer(() -> {
            //current pose
            Pose2d target = m_swerve.getPose();
            //need to drive backwards from the starting line
            if(CougarUtil.getAlliance() == Alliance.Red)
                target = CougarUtil.addDistanceToPoseRot(target, Rotation2d.kZero, 1);
            else
                target = CougarUtil.addDistanceToPoseRot(target, Rotation2d.k180deg, 1);
            return AutoBuilder.pathfindToPose(target, TunerConstants.kPathConstraints);
        });
    }
}
