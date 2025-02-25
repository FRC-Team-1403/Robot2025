package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import org.ejml.ops.DElementCoorBoolean;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {

    private double m_setpoint;
    private ElevatorSubsystem m_elevator;
    
    public ElevatorCommand(ElevatorSubsystem elevator, double setpoint) {
        m_elevator = elevator;
        m_setpoint = setpoint;

        addRequirements(m_elevator);
    }

    public void init() {}
    
    @Override
    public void execute() {
        Constants.Elevator.Setpoints.current = m_setpoint;
        m_elevator.moveToSetPoint(m_setpoint);
  }

  @Override
  public boolean isFinished() {
    return m_elevator.isAtSetpoint();
  }
}
