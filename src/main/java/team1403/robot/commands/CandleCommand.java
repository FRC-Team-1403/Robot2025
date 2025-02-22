package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.candle;

public class CandleCommand extends Command {
    private candle m_candle;
    private int m_r;
    private int m_g;
    private int m_b;

  public CandleCommand(candle candle, int r, int g, int b) {
    m_candle = candle;
    m_r = r;
    m_g = g;
    m_b = b;
    addRequirements(m_candle);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    m_candle.setLED(m_r, m_g, m_b);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}