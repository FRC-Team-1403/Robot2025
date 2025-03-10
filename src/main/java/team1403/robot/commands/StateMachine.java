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
    }

    @Override
    public void execute() {
        switch(Blackbox.robotState){
            case loading:
                m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.Source);
                m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Source);
                Blackbox.reefScoreLevel(ReefScoreLevel.drive);
                if(Blackbox.isCoralLoaded()) Blackbox.robotState = State.driving;
                break;
            case driving: 
                //if(Blackbox.reefLevel != Blackbox.reefLevel.drive)
                    //m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Drive);
                if(Blackbox.isAligning() && Blackbox.reefLevel != Blackbox.ReefScoreLevel.drive)
                    Blackbox.robotState = State.aligning;
                break;
            case aligning:
                Blackbox.placingState = Blackbox.pState.elevator;
                if (!Blackbox.isAligning())
                    Blackbox.robotState = State.placing;
                break;
            case placing:
                m_elevatorSubsystem.moveToSetpoint(Blackbox.getElevatorSetpointLevel(Blackbox.reefLevel));
                if(m_elevatorSubsystem.isAtSetpoint())
                    m_wristSubsystem.moveToSetpoint(Blackbox.getWristSetpointLevel(Blackbox.reefLevel));
                if(!Blackbox.isCoralLoaded()) {
                    m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Current);
                    if(m_wristSubsystem.isAtSetpoint())
                        Blackbox.robotState = State.exiting;
                }
                break;
                /*
                switch(Blackbox.reefLevel) {
                    case L1:
                        m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L1);
                        if(m_elevatorSubsystem.isAtSetpoint())
                            m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L1);
                        if(!m_intake.hasPiece()){
                            Blackbox.robotState = Blackbox.State.exiting;
                            System.out.println("hello1");
                        }
                        break;
                        // switch(Blackbox.placingState){
                        //     case elevator:
                        //         m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L1);
                        //         if(m_elevatorSubsystem.isAtSetpoint())
                        //             Blackbox.placingState = Blackbox.pState.wrist;
                        //         break;
                        //     case wrist:
                        //         m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L1);
                        //         if(m_wristSubsystem.isAtSetpoint())
                        //             Blackbox.placingState = Blackbox.pState.intake;
                        //         break;
                        //     case intake:
                        //         if(!m_intake.hasPiece())
                        //             Blackbox.robotState = Blackbox.State.exiting;
                        //         break;
                        // }
                        // break;
                    case L2:
                        m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L2);
                        if(m_elevatorSubsystem.isAtSetpoint())
                            m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L2);
                        if(!m_intake.hasPiece()){
                            Blackbox.robotState = Blackbox.State.exiting;
                            System.out.println("hello2");
                        }
                        break;
                        // switch(Blackbox.placingState){
                        //     case elevator:
                        //         m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L2);
                        //         if(m_elevatorSubsystem.isAtSetpoint())
                        //             Blackbox.placingState = Blackbox.pState.wrist;
                        //         break;
                        //     case wrist:
                        //         m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L2);
                        //         if(m_wristSubsystem.isAtSetpoint())
                        //             Blackbox.placingState = Blackbox.pState.intake;
                        //         break;
                        //     case intake:
                        //         if(!m_intake.hasPiece())
                        //             Blackbox.robotState = Blackbox.State.exiting;
                        //         break;
                        // }
                        //     break;
                    case L3:
                        m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L3);
                        if(m_elevatorSubsystem.isAtSetpoint())
                            m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L3);
                        if(!m_intake.hasPiece()){
                            Blackbox.robotState = Blackbox.State.exiting;
                            System.out.println("hello3");
                        }
                        break;
                        // switch(Blackbox.placingState){
                        //     case elevator:
                        //         m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L3);
                        //         if(m_elevatorSubsystem.isAtSetpoint())
                        //             Blackbox.placingState = Blackbox.pState.wrist;
                        //         break;
                        //     case wrist:
                        //         m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L3);
                        //         if(m_wristSubsystem.isAtSetpoint())
                        //             Blackbox.placingState = Blackbox.pState.intake;
                        //         break;
                        //     case intake:
                        //         if(!m_intake.hasPiece())
                        //             Blackbox.robotState = Blackbox.State.exiting;
                        //         break;
                        // }
                        //     break;
                    case L4:
                        m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L4);
                        if(m_elevatorSubsystem.isAtSetpoint())
                            m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L4);
                        if(!m_intake.hasPiece()){
                            Blackbox.robotState = Blackbox.State.exiting;
                            System.out.println("hello4");
                        }
                        break;
                    //     switch(Blackbox.placingState){
                    //         case elevator:
                    //             m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L4);
                    //             if(m_elevatorSubsystem.isAtSetpoint())
                    //                 Blackbox.placingState = Blackbox.pState.wrist;
                    //             break;
                    //         case wrist:
                    //             m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L4);
                    //             if(m_wristSubsystem.isAtSetpoint())
                    //                 Blackbox.placingState = Blackbox.pState.intake;
                    //             break;
                    //         case intake:
                    //             if(!m_intake.hasPiece())
                    //                 Blackbox.robotState = Blackbox.State.exiting;
                    //             break;
                    //     }
                    // break;
                    case drive:
                    default:
                        m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Source);
                        m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.Current);
                        break;
                }*/
            case exiting: {
                System.out.println("HI");
                // Blackbox.placingState = Blackbox.pState.drive;
                m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.Current);
                //m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Drive);
                break;
            }
            case ManualElevator: 
                m_elevatorSubsystem.moveToSetpoint(m_elevatorSubsystem.getSetpoint() - 
                    MathUtil.applyDeadband(m_op.getRightY(), 0.05) * Constants.kLoopTime * 32);
                m_wristSubsystem.moveToSetpoint(m_wristSubsystem.getSetpoint() + 
                    MathUtil.applyDeadband(m_op.getLeftY(), 0.05) * Constants.kLoopTime / 7.0);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
