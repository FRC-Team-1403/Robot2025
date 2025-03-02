package team1403.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
    }

    @Override
    public void execute(){
        switch(Blackbox.robotState){
            case loading: {
                m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.Source);
                m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Source / 360.);
                Blackbox.reefScoreLevel(ReefScoreLevel.drive);
                if(Blackbox.isCoralLoaded()) Blackbox.robotState = State.driving;
                break;
            } case driving: {
                switch(Blackbox.reefLevel) {
                    case drive: {
                        m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Source / 360.);
                        break;
                    } case L1: {
                        m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L1 / 360.);
                        break;
                    } case L2: {
                        m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L2 / 360.);
                        break;
                    } case L3: {
                        m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L3 / 360.);
                        break;
                    } case L4: {
                        m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L4 / 360.);
                        break;
                    }
                }
                if(Blackbox.isAligning())
                    Blackbox.robotState = State.aligning;
                break;
            } case aligning: {
                if(Blackbox.getCloseAlign(m_swerve.getPose())){
                    switch(Blackbox.reefLevel) {
                        case L1: {
                            m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L1);
                            break;
                        } case L2: {
                            m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L2);
                            break;
                        } case L3: {
                            m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L3);
                            break;
                        } case L4: {
                            m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L4);
                            break;
                        }
                        case drive:
                        default: {
                            m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.Current);
                            break;
                        }
                    }
                }
                if (!Blackbox.isAligning())
                    Blackbox.robotState = State.placing;
                break;
            } case placing: {
                if(!Blackbox.isCoralLoaded()){
                    m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Current / 360.);
                    if (m_wristSubsystem.isAtSetpoint())
                        m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.Current);
                    //if(m_wristSubsystem.isAtSetpoint()) Blackbox.robotState = State.loading;
                }
                break;
            } case ManualElevator: {
                m_elevatorSubsystem.moveToSetpoint(m_elevatorSubsystem.getSetpoint() - 
                    MathUtil.applyDeadband(m_op.getRightY(), 0.05) * Constants.kLoopTime * 32);
                m_wristSubsystem.moveToSetpoint(m_wristSubsystem.getSetpoint() + 
                    MathUtil.applyDeadband(m_op.getLeftY(), 0.05) * Constants.kLoopTime / 7.0);
                break;
            }
        }
        Logger.recordOutput("State", Blackbox.robotState.toString());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
