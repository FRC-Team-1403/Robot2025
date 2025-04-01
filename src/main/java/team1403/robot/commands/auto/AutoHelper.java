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
                Commands.parallel(
                    AutoUtil.loadPathPlannerPath("OneP Center", m_swerve),
                    Commands.sequence(
                        Commands.waitSeconds(0.75),
                        NamedCommands.getCommand("CoralL4"))
                ),
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

    public static Command getThreePieceSideProc(SwerveSubsystem m_swerve) {
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

    

    public static Command getThreePieceBackProc(SwerveSubsystem m_swerve) {
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
                Commands.waitSeconds(0.1),
                AutoUtil.loadPathPlannerPath("Proc Two Piece Part 2 test", m_swerve),
                //Commands.waitSeconds(0.05),
                // alignToStartingPose(m_swerve, "Proc2P Part 2"),
                NamedCommands.getCommand("WaitForCoral"),
                AutoUtil.loadPathPlannerPath("Proc Part 3 Back", m_swerve),
                NamedCommands.getCommand("CoralL4"),
                NamedCommands.getCommand("ReefAlignL"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                Commands.parallel(
                    NamedCommands.getCommand("Loading"),
                // Commands.waitSeconds(0.05),
                //alignToStartingPose(m_swerve, "3p Part 4"),
                    AutoUtil.loadPathPlannerPath("3p Part 4", m_swerve)
                ),
                NamedCommands.getCommand("WaitForCoral"),
                AutoUtil.loadPathPlannerPath("Proc Part 5 Back", m_swerve),
                NamedCommands.getCommand("CoralL4"),
                NamedCommands.getCommand("ReefAlignR"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                Commands.parallel(
                    NamedCommands.getCommand("Loading"),
                    AutoUtil.loadPathPlannerPath("Proc part 6 back", m_swerve)
                )
                
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }

    public static Command getThreePieceBackNonProc(SwerveSubsystem m_swerve) {
        try {
            return Commands.sequence(
                Commands.parallel(
                    AutoUtil.loadPathPlannerPath("Non Proc 3P Part 1", m_swerve, true),
                    Commands.sequence(
                        Commands.waitSeconds(0.75),
                        NamedCommands.getCommand("CoralL4"))
                ),
                NamedCommands.getCommand("ReefAlignL"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                NamedCommands.getCommand("Loading"),
                Commands.waitSeconds(0.1),
                AutoUtil.loadPathPlannerPath("Non Proc 3P Part 2", m_swerve),
                NamedCommands.getCommand("WaitForCoral"),
                AutoUtil.loadPathPlannerPath("Non Proc 3P Part 3", m_swerve),
                NamedCommands.getCommand("CoralL4"),
                NamedCommands.getCommand("ReefAlignR"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                Commands.parallel(
                    NamedCommands.getCommand("Loading"),
                    AutoUtil.loadPathPlannerPath("Non Proc 3P Part 4", m_swerve)
                ),
                NamedCommands.getCommand("WaitForCoral"),
                AutoUtil.loadPathPlannerPath("Non Proc 3P Part 5", m_swerve),
                NamedCommands.getCommand("CoralL4"),
                NamedCommands.getCommand("ReefAlignL"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                Commands.parallel(
                    NamedCommands.getCommand("Loading"),
                    AutoUtil.loadPathPlannerPath("Non Proc 3P Part 6", m_swerve)
                )
                
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
                AutoUtil.loadPathPlannerPath("Proc2P Part 1", m_swerve, true), //SCORES PIECE 1                       
                NamedCommands.getCommand("CoralL4"),
                NamedCommands.getCommand("ReefAlignR"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                NamedCommands.getCommand("Loading"),
                AutoUtil.loadPathPlannerPath("Proc Two Piece Part 2 test", m_swerve), //LOADING  STATION
                NamedCommands.getCommand("WaitForCoral"),
                AutoUtil.loadPathPlannerPath("Proc2P Part 3", m_swerve), //SCORES PIECE 2 
                NamedCommands.getCommand("CoralL4"),
                NamedCommands.getCommand("ReefAlignL"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                NamedCommands.getCommand("Loading"),
                AutoUtil.loadPathPlannerPath("Proc2P Part 4 test", m_swerve) // LOADING STATION 

            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }

    public static Command getTwoPieceProc_algaeRemoval(SwerveSubsystem m_swerve) {
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
                    AutoUtil.loadPathPlannerPath("Proc2P +Algae Part 3", m_swerve),
                    Commands.sequence(
                        Commands.waitSeconds(0.75),
                        NamedCommands.getCommand("CoralL4")),
                        Commands.waitSeconds(0.25)
                ),
                NamedCommands.getCommand("ReefAlignL"),
                NamedCommands.getCommand("WaitForSetpoint"),
                NamedCommands.getCommand("CoralScore"),
                NamedCommands.getCommand("Loading")
                //Align to Center Command
                //Take out algae
            );
        } catch (Exception e) {
            System.err.println("Could not load auto: " + e.getMessage());
            return Commands.none();
        }
    }

    
}
