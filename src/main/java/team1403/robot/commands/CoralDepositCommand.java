package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.*;
import team1403.robot.commands.StateMachine;

public class CoralDepositCommand extends Command {

    private IntakeSubsystem m_intake;
    private WristSubsystem m_wrist;
    private int m_level;

    public CoralDepositCommand(IntakeSubsystem intake, WristSubsystem wrist, int level) {
        m_intake = intake;
        m_wrist = wrist;
        m_level = level;
        addRequirements(m_intake);
  }

    @Override
    public void initialize() {
        // wrist setpoint same for L2 & L3
        if(m_level == 1) m_wrist.setWristAngle(Constants.Wrist.Setpoints.L1Setpoint);
        if(m_level == 2 || m_level == 3) m_wrist.setWristAngle(Constants.Wrist.Setpoints.L2Setpoint);
        if(m_level == 4) m_wrist.setWristAngle(Constants.Wrist.Setpoints.L4Setpoint);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        if(m_intake.getDistance() > 0) return true;
        return false;
    }
}
