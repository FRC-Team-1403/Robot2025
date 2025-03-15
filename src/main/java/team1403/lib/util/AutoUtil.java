package team1403.lib.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;

public class AutoUtil {
    
  //resets odometery
  public static Command loadChoreoPath(String name, SwerveSubsystem swerve) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
      Command cmd = AutoBuilder.followPath(path);
      return Commands.sequence(Commands.runOnce(() -> {
          Pose2d startPose = CougarUtil.shouldMirrorPath() ? 
            path.flipPath().getStartingDifferentialPose() : 
            path.getStartingDifferentialPose();
          swerve.resetOdometry(startPose);
      }, swerve), cmd);
    } catch (Exception e) {
      System.err.println("Failed to load choreo auto: " + e.getMessage());
      return null;
    }
  }

  public static Pose2d getStartingPose(PathPlannerPath path) {
    PathPlannerPath mirrored = CougarUtil.shouldMirrorPath() ? path.flipPath() : path;
    Pose2d startPose = mirrored.getStartingDifferentialPose();
    if(AutoBuilder.isHolonomic()) 
      startPose = CougarUtil.createPose2d(startPose, mirrored.getIdealStartingState().rotation());
    return startPose;
  }

  public static Pose2d getStartingPose(String name) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
      return getStartingPose(path);
    } catch (Exception e) {
      System.err.println("Failed to load pathplanner path: " + e.getMessage());
      return null;
    }
  }

  //resets odometery
  public static Command loadPathPlannerPath(String name, SwerveSubsystem swerve, boolean reset) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
      Command cmd = AutoBuilder.followPath(path);
      return Commands.sequence(Commands.runOnce(() -> {
          if (reset) swerve.resetOdometry(getStartingPose(path));
      }, swerve), cmd);
    } catch (Exception e) {
      System.err.println("Failed to load pathplanner path: " + e.getMessage());
      return null;
    }
  }

  public static Command loadPathPlannerPath(String name, SwerveSubsystem swerve) {
    return loadPathPlannerPath(name, swerve, false);
  }

  public static Command pathFindToPose(Pose2d target) {
    return AutoBuilder.pathfindToPose(target, TunerConstants.kPathConstraints);
  }

  public static Command pathFindtoPath(PathPlannerPath path) {
    return AutoBuilder.pathfindThenFollowPath(path, TunerConstants.kPathConstraints);
  }
}
