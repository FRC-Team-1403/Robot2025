package team1403.lib.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitUntilDebounced extends Command {

    private final BooleanSupplier m_condition;
    private final Debouncer m_debouncer;

    public WaitUntilDebounced(BooleanSupplier input, double debounceTime) {
        m_condition = input;
        m_debouncer = new Debouncer(debounceTime, DebounceType.kRising);
    }

    @Override
    public boolean isFinished() {
        return m_debouncer.calculate(m_condition.getAsBoolean());
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
    
}
