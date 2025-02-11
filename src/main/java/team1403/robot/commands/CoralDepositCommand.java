package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.*;
import team1403.robot.subsystems.Blackbox.ReefScoreLevel;
import team1403.robot.subsystems.Blackbox.ReefSelect;
import team1403.robot.commands.StateMachine;

public class CoralDepositCommand extends Command {


    private ReefScoreLevel m_level;
    private ReefSelect m_select;

    //edit to just passthrough enum (its static)
    public CoralDepositCommand(ReefScoreLevel level, ReefSelect select) {
        m_level = level;
        m_select = select;
  }

    @Override
    public void initialize() {
        // wrist setpoint same for L2 & L3
        Blackbox.reefSelect(m_select);
        Blackbox.reefScoreLevel(m_level);
        Blackbox.setTrigger(true);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return !Blackbox.getTrigger();
    }
}
