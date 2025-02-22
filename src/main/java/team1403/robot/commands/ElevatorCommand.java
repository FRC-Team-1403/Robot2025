package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import org.ejml.ops.DElementCoorBoolean;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Elevator;

public class ElevatorCommand extends Command {

    private BooleanSupplier m_zero;
    private BooleanSupplier m_low;
    private BooleanSupplier m_mid;
    private BooleanSupplier m_high;
    private double setpoint;
    private Elevator m_elevator;
    
    public ElevatorCommand(Elevator elevator, BooleanSupplier zero, BooleanSupplier low, BooleanSupplier mid, BooleanSupplier high) {
        m_elevator = elevator;
        m_zero = zero;
        m_low = low;
        m_mid = mid;
        m_high = high;
        setpoint = 0;

        addRequirements(m_elevator);
    }

    public void init() {}
    
    @Override
    public void execute() {
        if (m_zero.getAsBoolean()) {
          setpoint = Constants.Elevator.Setpoints.L1;
          Constants.Elevator.Setpoints.current = Constants.Elevator.Setpoints.L1 / 360.0;
        }
        else if (m_low.getAsBoolean()) {
          setpoint = Constants.Elevator.Setpoints.L2;
          Constants.Elevator.Setpoints.current = Constants.Elevator.Setpoints.L2 / 360.0;
        }
        else if (m_mid.getAsBoolean()) {
          setpoint = Constants.Elevator.Setpoints.L3;
          Constants.Elevator.Setpoints.current = Constants.Elevator.Setpoints.L3 / 360.0;
        }
        else if (m_high.getAsBoolean()) {
          setpoint = Constants.Elevator.Setpoints.L4;
          Constants.Elevator.Setpoints.current = Constants.Elevator.Setpoints.L4 / 360.0;
        }
        m_elevator.moveToSetPoint(setpoint);
    }
}
