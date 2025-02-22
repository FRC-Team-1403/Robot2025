package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.candle;

public class CandleCommand extends Command {
    private candle m_candle;

  public CandleCommand(candle candle) {
    m_candle = candle;
  }

  @Override
  public void initialize() {
    m_candle.setLED();
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
