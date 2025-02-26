package team1403.robot.commands;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team1403.lib.util.AutoUtil;
import team1403.lib.util.CougarUtil;
import team1403.robot.Constants;
import team1403.robot.subsystems.*;
import team1403.robot.subsystems.Blackbox.ReefScoreLevel;
import team1403.robot.subsystems.Blackbox.ReefSelect;
import team1403.robot.subsystems.Blackbox.State;
import team1403.robot.swerve.SwerveSubsystem;

public class StateMachine extends Command {
    private WristSubsystem m_wristSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;
    
   

    public StateMachine(WristSubsystem wrist, ElevatorSubsystem elevator){
        m_wristSubsystem = wrist;
        m_elevatorSubsystem = elevator;

        addRequirements(m_wristSubsystem, m_elevatorSubsystem);
    }

    @Override
    public void execute(){
        switch(Blackbox.robotState){
            case loading: {
                Blackbox.setCloseAlign(false);
                m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.Source);
                m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Source);
                Blackbox.reefScoreLevel(ReefScoreLevel.drive);
                if(Blackbox.isCoralLoaded()) Blackbox.robotState = State.driving;
                break;
            } case driving: {
                switch(Blackbox.reefLevel) {
                    case drive: {
                        m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Source);
                    } case L1: {
                        m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L1);
                    } case L2: {
                        m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L2);
                    } case L3: {
                        m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L3);
                    } case L4: {
                        m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.L4);
                    }
                }
                if(Blackbox.isAligning())
                    Blackbox.robotState = State.aligning;
                break;
            } case aligning: {
                if(Blackbox.getCloseAlign()){
                    switch(Blackbox.reefLevel) {
                        case L1: {
                            m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L1);
                        } case L2: {
                            m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L2);
                        } case L3: {
                            m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L3);
                        } case L4: {
                            m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.Setpoints.L4);
                        }
                    }

                }
                if (!Blackbox.isAligning())
                    Blackbox.robotState = State.placing;
                break;
            } case placing: {
                if(!Blackbox.isCoralLoaded()){
                    m_wristSubsystem.moveToSetpoint(Constants.Wrist.Setpoints.Current);
                    if(m_wristSubsystem.isAtSetpoint()) Blackbox.robotState = State.loading;
                }
                break;
            }
        }
        // Logger.recordOutput("current pose111", currentPose);
        // Logger.recordOutput("targ111et", target);
        Logger.recordOutput("State", Blackbox.robotState.toString());
    }
}
