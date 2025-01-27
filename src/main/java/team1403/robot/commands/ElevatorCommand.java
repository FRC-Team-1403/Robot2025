package team1403.robot.commands;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Elevator;
import team1403.robot.Constants;
import team1403.robot.MotionProfiler;


public class ElevatorCommand extends Command {
  private Elevator m_elevator;
  private double currentPos;
  MotionProfiler profiler;

  public ElevatorCommand(Elevator elevator, double m_currentPos, double m_currentMotorOutput) {
    m_elevator = elevator;
    currentPos = m_currentPos;

    profiler = new MotionProfiler(currentPos);
    
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    profiler.moveToSetPoint(70);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
