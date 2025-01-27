package team1403.robot.commands;
import java.util.function.BooleanSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Elevator;
import team1403.robot.Constants;
import team1403.robot.MotionProfiler;


public class ElevatorCommand extends Command {
  private Elevator m_elevator;
  private double currentPos;
  MotionProfiler profiler;
  private BooleanSupplier m_first;
  private BooleanSupplier m_second;
  private BooleanSupplier m_third;
  private BooleanSupplier m_down;
  private double setpoint;

  public ElevatorCommand(Elevator elevator, BooleanSupplier first, BooleanSupplier second, BooleanSupplier third, BooleanSupplier down) {
    m_elevator = elevator;
    setpoint = 0;
    currentPos = 0;
    m_first = first;
    m_second = second;
    m_third = third;
    m_down = down;

    profiler = new MotionProfiler(currentPos);
    
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_down.getAsBoolean()) {
      setpoint = 0;
    }
    else if (m_first.getAsBoolean()) {
      setpoint = 10;
    }
    else if (m_second.getAsBoolean()) {
      setpoint = 40;
    }
    else if (m_third.getAsBoolean()) {
      setpoint = 70;
    }
    profiler.moveToSetPoint(setpoint);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
