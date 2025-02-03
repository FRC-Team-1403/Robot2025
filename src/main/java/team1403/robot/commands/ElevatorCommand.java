package team1403.robot.commands;
import java.util.function.BooleanSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Elevator;
import team1403.robot.Constants;

public class ElevatorCommand extends Command {
  private Elevator m_elevator;
  private BooleanSupplier m_first;
  private BooleanSupplier m_second;
  private BooleanSupplier m_third;
  private BooleanSupplier m_down;
  private double setpoint;

  public ElevatorCommand(Elevator elevator, BooleanSupplier first, BooleanSupplier second, BooleanSupplier third, BooleanSupplier down) {
    m_elevator = elevator;
    setpoint = 0;
    m_first = first;
    m_second = second;
    m_third = third;
    m_down = down;
    
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    // m_elevator.MotionProfiler();
    m_down = () -> true;
  }

  @Override
  public void execute() {
    // if (m_down.getAsBoolean() && setpoint != Constants.Elevator.down) {
    //   setpoint = Constants.Elevator.down; // 0
    // }
    // else if (m_first.getAsBoolean() && setpoint != Constants.Elevator.first) {
    //   setpoint = Constants.Elevator.first; // 5
    // }
    // else if (m_second.getAsBoolean() && setpoint != Constants.Elevator.second) {
    //   setpoint = Constants.Elevator.second; // 10
    // }
    // else if (m_third.getAsBoolean() && setpoint != Constants.Elevator.third) {
    //   setpoint = Constants.Elevator.third; // 20
    // }
    setpoint = 5;
    m_elevator.moveToSetPoint(setpoint);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
