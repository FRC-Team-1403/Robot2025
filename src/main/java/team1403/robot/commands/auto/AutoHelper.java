package team1403.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team1403.lib.util.AutoUtil;
import team1403.lib.util.CougarUtil;
import team1403.robot.commands.AlignCommand;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;

public class AutoHelper {
    private static Command alignToStartingPose(SwerveSubsystem swerve, String name) {
        return swerve.defer(() -> new AlignCommand(swerve, AutoUtil.getStartingPose(name)));
    }
    
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

    public static Command getOnePCenter(SwerveSubsystem m_swerve) {
        try {
            return Commands.sequence(
                Commands.waitSeconds(0.2), //wait for state machine to reach correct state
                AutoUtil.loadPathPlannerPath("OneP Center", m_swerve),
                NamedCommands.getCommand("CoralL4"),
                NamedCommands.getCommand("ReefAlignR"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                NamedCommands.getCommand("Loading")
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }

    public static Command getThreePieceProc(SwerveSubsystem m_swerve) {
        try {
            return Commands.sequence(
                Commands.waitSeconds(1),
                AutoUtil.loadPathPlannerPath("Proc2P Part 1", m_swerve, true),
                NamedCommands.getCommand("CoralL4"),
                NamedCommands.getCommand("ReefAlignR"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                NamedCommands.getCommand("Loading"),
                alignToStartingPose(m_swerve, "Proc2P Part 2"),
                AutoUtil.loadPathPlannerPath("Proc2P Part 2", m_swerve),
                NamedCommands.getCommand("WaitForCoral"),
                AutoUtil.loadPathPlannerPath("Proc2P Part 3", m_swerve),
                NamedCommands.getCommand("CoralL4"),
                NamedCommands.getCommand("ReefAlignR"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                NamedCommands.getCommand("Loading"),
                alignToStartingPose(m_swerve, "Proc2P Part 4"),
                AutoUtil.loadPathPlannerPath("Proc2P Part 4", m_swerve),
                NamedCommands.getCommand("WaitForCoral"),
                AutoUtil.loadPathPlannerPath("Proc2P Part 5", m_swerve),
                NamedCommands.getCommand("CoralL4"),
                NamedCommands.getCommand("ReefAlignL"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                NamedCommands.getCommand("Loading")
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            
            return Commands.none();
        }
    }

    public static Command testAutoAlign(SwerveSubsystem m_swerve){
        try{
            return Commands.sequence(
                Commands.waitSeconds(1.0),
                NamedCommands.getCommand("CoralL4"),
                NamedCommands.getCommand("ReefAlignL"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                NamedCommands.getCommand("Loading")
            );
        } 
        catch(Exception e){
            System.err.println("Could not load");
            return Commands.none();
        }
  
    }
    public static Command getTwoPieceProc(SwerveSubsystem m_swerve) {
        try {
            return Commands.sequence(
                Commands.parallel(
                    AutoUtil.loadPathPlannerPath("Proc2P Part 1", m_swerve, true),
                    Commands.sequence(
                        Commands.waitSeconds(0.75),
                        NamedCommands.getCommand("CoralL4"))
                ),
                NamedCommands.getCommand("ReefAlignR"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                NamedCommands.getCommand("Loading"),
                alignToStartingPose(m_swerve, "Proc2P Part 2"),
                AutoUtil.loadPathPlannerPath("Proc2P Part 2", m_swerve),
                NamedCommands.getCommand("WaitForCoral"),
                Commands.parallel(
                    AutoUtil.loadPathPlannerPath("Proc2P Part 3", m_swerve),
                    Commands.sequence(
                        Commands.waitSeconds(0.75),
                        NamedCommands.getCommand("CoralL4")),
                        Commands.waitSeconds(0.25)
                ),
                NamedCommands.getCommand("ReefAlignL"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore")
                // NamedCommands.getCommand("Loading")
           
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }


    public static Command getTwoPieceProcTest(SwerveSubsystem m_swerve) {
        try {
            return Commands.sequence(
                Commands.parallel(
                    AutoUtil.loadPathPlannerPath("Proc2P Part 1", m_swerve, true),
                    Commands.sequence(
                        Commands.waitSeconds(0.75),
                        NamedCommands.getCommand("CoralL4"))
                ),
                NamedCommands.getCommand("ReefAlignR"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                Commands.waitSeconds(0.2),
                Commands.parallel(
                    Commands.sequence(
                        Commands.waitSeconds(1),
                        NamedCommands.getCommand("Loading")
                    ),
                    AutoUtil.loadPathPlannerPath("Proc Two Piece Part 2 test", m_swerve)
                ),
                NamedCommands.getCommand("WaitForCoral"),
                Commands.parallel(
                    AutoUtil.loadPathPlannerPath("Proc2P Part 3", m_swerve),
                    Commands.sequence(
                        Commands.waitSeconds(0.75),
                        NamedCommands.getCommand("CoralL4")),
                        Commands.waitSeconds(0.25)
                ),
                NamedCommands.getCommand("ReefAlignL"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                Commands.waitSeconds(0.2),
                Commands.parallel(
                    Commands.sequence(
                        Commands.waitSeconds(1),
                        NamedCommands.getCommand("Loading")
                    ),
                    AutoUtil.loadPathPlannerPath("Proc2P Part 4 test", m_swerve)
                )
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }
}
