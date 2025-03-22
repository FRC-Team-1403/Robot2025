package team1403.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team1403.robot.Constants;
import team1403.robot.subsystems.*;
import team1403.robot.subsystems.Blackbox.ReefScoreLevel;
import team1403.robot.subsystems.Blackbox.State;
import team1403.robot.swerve.SwerveSubsystem;

public class StateMachine extends Command {
    private WristSubsystem m_wristSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;
    //only used for getting position
    private SwerveSubsystem m_swerve;
    private XboxController m_op;

    public StateMachine(WristSubsystem wrist, ElevatorSubsystem elevator, SwerveSubsystem drivetrain, XboxController op){
        m_wristSubsystem = wrist;
        m_elevatorSubsystem = elevator;
        m_swerve = drivetrain;
        m_op = op;

        addRequirements(m_wristSubsystem, m_elevatorSubsystem);
    }

    @Override
    public void initialize() {
        Blackbox.robotState = Blackbox.State.loading;
        Blackbox.reefScoreLevel(ReefScoreLevel.drive);
    }

    @Override
    public void execute() {
        switch(Blackbox.robotState){
            case loading:     
                m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Source);
                if (m_wristSubsystem.isAtSetpoint()) {
                    m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.Source); 
                    // Blackbox.reefScoreLevel(ReefScoreLevel.drive);
                    if (Blackbox.isCoralLoaded()) Blackbox.robotState = State.driving;
                }
                break;
            case driving: 
                if(Blackbox.reefLevel != Blackbox.reefLevel.drive)
                    m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Drive);
                if(Blackbox.isAligning() && Blackbox.reefLevel != Blackbox.ReefScoreLevel.drive)
                    Blackbox.robotState = State.aligning;
                break;
            case aligning:
                if(Blackbox.isAligning() && Blackbox.getCloseAlign(m_swerve.getPose()) && Blackbox.reefLevel == Blackbox.ReefScoreLevel.L4)
                    Blackbox.robotState = State.placing;
                if(!Blackbox.isAligning())
                    Blackbox.robotState = State.placing;
                break;
            case placing:
                m_elevatorSubsystem.moveToSetpoint(Blackbox.getElevatorSetpointLevel(Blackbox.reefLevel));
                if(m_elevatorSubsystem.isAtSetpoint())
                {
                    m_wristSubsystem.moveToSetpoint(Blackbox.getWristSetpointLevel(Blackbox.reefLevel));
                    if(!Blackbox.isCoralLoaded()) {
                        m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Drive);
                        if(m_wristSubsystem.isAtSetpoint())
                            Blackbox.robotState = State.exiting;
                    }
                }
                break;
            case exiting: {
                //System.out.println("HI");
                // Blackbox.placingState = Blackbox.pState.drive;
                m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.Current);
                Blackbox.reefScoreLevel(ReefScoreLevel.drive);
                //m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Drive);
                break;
            }
            case ManualElevator:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
