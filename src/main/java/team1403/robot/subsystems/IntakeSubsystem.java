package team1403.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import team1403.robot.Constants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax m_intakeMotor;
    private CANrange m_CANRange;

    public IntakeSubsystem() {
        m_intakeMotor = new SparkMax(Constants.CanBus.intakeMotorID, MotorType.kBrushless);
        configMotors();
        m_CANRange = new CANrange(Constants.CanBus.CANRangeID);
    }

      private void configMotors() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig
        .idleMode(IdleMode.kBrake);

    m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

    public void setIntakeMotorSpeed(double speed) {
        if (!hasPiece() || (hasPiece() && speed < 0)) {
            m_intakeMotor.set(speed);
        }
    }
    
    public double getIntakeSpeed() {
        return m_intakeMotor.get();
    }

    public boolean hasPiece() {
        return getDistance() < 0.4;
    }

    //TODO add in the distance threholds
    public double getDistance() {
        return m_CANRange.getDistance(true).getValue().in(Meters);
    }

    public void periodic() {
        if (!hasPiece() && getIntakeSpeed() > 0.0) {
            setIntakeMotorSpeed(0);
        }

        Logger.recordOutput("Distance", getDistance());
        Logger.recordOutput("Has Piece", hasPiece());
    }
}