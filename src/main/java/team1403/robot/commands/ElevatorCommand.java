package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import org.ejml.ops.DElementCoorBoolean;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {

    private BooleanSupplier m_zero;
    private BooleanSupplier m_low;
    private BooleanSupplier m_mid;
    private BooleanSupplier m_high;
    private double setpoint;
    private ElevatorSubsystem m_elevator;
    
    public ElevatorCommand(ElevatorSubsystem elevator, BooleanSupplier zero, BooleanSupplier low, BooleanSupplier mid, BooleanSupplier high) {
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
        }
        else if (m_low.getAsBoolean()) {
          setpoint = Constants.Elevator.Setpoints.L2;
        }
        else if (m_mid.getAsBoolean()) {
          setpoint = Constants.Elevator.Setpoints.L3;
        }
        else if (m_high.getAsBoolean()) {
          setpoint = Constants.Elevator.Setpoints.L4;
        }
        Constants.Elevator.Setpoints.current = setpoint;
        m_elevator.moveToSetPoint(setpoint);
  }
}
