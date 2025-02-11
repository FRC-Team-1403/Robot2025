package team1403.robot.commands;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team1403.lib.util.AutoUtil;
import team1403.robot.Constants;
import team1403.robot.subsystems.*;
import team1403.robot.subsystems.Blackbox.ReefSelect;
import team1403.robot.swerve.SwerveSubsystem;

public class StateMachine extends Command {
    private IntakeSubsystem m_intakeSubsystem;
    private WristSubsystem m_wristSubsystem;
    private Elevator m_elevatorSubsystem;
    private SwerveSubsystem m_swerve;
    private Supplier<Command> m_vibrationCmd;
    private static State m_state;
    
    public enum State{
        loading,
        driving,
        aligning,
        placing
    }

    public StateMachine(IntakeSubsystem intake, WristSubsystem wrist, Elevator elevator, SwerveSubsystem swerve, Supplier<Command> vibrationCmd){
        m_intakeSubsystem = intake;
        m_wristSubsystem = wrist;
        m_elevatorSubsystem = elevator;
        m_swerve = swerve;
        m_vibrationCmd = vibrationCmd;

        addRequirements(m_intakeSubsystem, m_wristSubsystem, m_elevatorSubsystem);
    }

    public void execute(){
        switch(m_state){
            case loading: {
                m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.source);
                //TODO align with april tag and add option to move sides
                if(Blackbox.isCoralLoaded()) m_state = State.driving;
                break;
            } case driving: {
                m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.down);
                //nested switch is dependent on blackbox variable, set by controller during teleop and in pathplanner during auto
                switch(Blackbox.getReefScoreLevel())
                {
                    case L1: {
                        m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L1);
                        m_state = State.aligning;
                        break;
                    } case L2: {
                        m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L2);
                        m_state = State.aligning;
                        break;
                    } case L3: {
                        m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L3);
                        m_state = State.aligning;
                        break;
                    } case L4: {
                        m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L4);
                        m_state = State.aligning;
                        break;
                    }
                }
                break;
            } case aligning: {
                //TODO add logic for aligning based on coral position (CAN range)
                //trust me bro it probably works
                // sends signal from button press or auto command
                if(Blackbox.isAligning()) {
                    // checks right/left
                    if(Blackbox.getReefSelect().equals(ReefSelect.RIGHT)) {
                        //auto aligns to ideal pose (distance and direction)
                        new DeferredCommand(() -> {Blackbox.reefSelect(ReefSelect.RIGHT);
                            Pose2d currentPose = m_swerve.getPose();
                            Pose2d target = Blackbox.getNearestAlignPositionReef(currentPose);
                            if (target == null) return Commands.none();
                            return Commands.sequence(
                                AutoUtil.pathFindToPose(target),
                                new AlignCommand(m_swerve, target).finallyDo((interrupted) -> {
                                    //vibration cmd just calls ControllerVibrationCommand
                                    if(!interrupted) m_vibrationCmd.get().schedule();
                                }),
                                //resets aligning trigger
                                Blackbox.setAligningCmd(false, ReefSelect.LEFT)
                            ); 
                            //cancels if trigger is released
                            //TODO figure out if onlyWhile will work for auto command
                        }, Set.of(m_swerve)).onlyWhile(() -> Blackbox.isAligning()).schedule();
                        // same thing but if its left side
                    } else {
                            new DeferredCommand(() -> {Blackbox.reefSelect(ReefSelect.LEFT);
                                Pose2d currentPose = m_swerve.getPose();
                                Pose2d target = Blackbox.getNearestAlignPositionReef(currentPose);
                                if (target == null) return Commands.none();
                                return Commands.sequence(
                                    AutoUtil.pathFindToPose(target),
                                    new AlignCommand(m_swerve, target).finallyDo((interrupted) -> {
                                        if(!interrupted) m_vibrationCmd.get().schedule();
                                    }),
                                    Blackbox.setAligningCmd(false, ReefSelect.LEFT)
                                ); 
                            }, Set.of(m_swerve)).onlyWhile(() -> Blackbox.isAligning()).schedule();
                        }
                    m_state = State.placing;
                }
                break;
            } case placing: {
                
                if(!Blackbox.isCoralLoaded()) m_state = State.loading;
                break;
            }
        }
    }
    
}
