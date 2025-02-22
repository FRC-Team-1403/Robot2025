package team1403.lib.util;

import static edu.wpi.first.units.Units.Meter;

import java.util.HashMap;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;


public class C {
    //!variables
    private String InputType = null;
    private String RangeType = null;
    private String MotorType_ = null;
    private HashMap<String, Object> aliases = new HashMap<>();

    //!motors
    public void setMotor(double speed, int id) {
        if ("SparkMax".equals(MotorType_)) {
            SparkMax m_motor = new SparkMax(id, MotorType.kBrushless);
            m_motor.set(speed);
            m_motor.close();
        }
    }

    public void setMotor(double speed, String name, boolean close) {
        Object motor = aliases.get(name);

        if (motor instanceof SparkMax) {
            SparkMax m_motor = (SparkMax) motor;
            m_motor.set(speed);
            if (close == true) {
                m_motor.close();
            }
        }
    }

    public void setBoundsMotor(double angle1, double angle2, int id) {

    }

    public void setBoundsMotor(double angle1, double angle2, String name) {
        
    }

    public void setMotorName(int id, String name) {
        if (aliases.get(name) == null) {
            if ("SparkMax".equals(MotorType_)) {
                SparkMax motor = new SparkMax(id, MotorType.kBrushless);
                aliases.put(name, motor);
            }
        }
    }

    //!encoders

    //!input/output
    public boolean isTriggered(int id) {
        boolean value = false;

        if ("DigitalInput".equals(InputType)) {
            DigitalInput m_I_O = new DigitalInput(id);
            value = m_I_O.get();
            m_I_O.close();
        }
        return value;
    }

    public boolean isTriggered(String name, boolean close) {
        Object IO = aliases.get(name);
        boolean value = false;

        if (IO instanceof DigitalInput) {
            DigitalInput m_IO = (DigitalInput) IO;
            value = m_IO.get();
            if (close == true) {
                m_IO.close();
            }
        }
        return value;
    }

    public boolean isTriggeredFlipped(int id) {
        boolean value = false;

        if ("DigitalInput".equals(InputType)) {
            DigitalInput m_I_O = new DigitalInput(id);
            value = !m_I_O.get();
            m_I_O.close();
        }
        return value;
    }

    public boolean isTriggeredFlipped(String name, boolean close) {
        Object IO = aliases.get(name);
        boolean value = false;

        if (IO instanceof DigitalInput) {
            DigitalInput m_IO = (DigitalInput) IO;
            value = !m_IO.get();
            if (close == true) {
                m_IO.close();
            }
        }
        return value;
    }

    public void setIOName(int id, String name) {
        if (aliases.get(name) == null) {
            if ("DigitalInput".equals(InputType)) {
                DigitalInput IO = new DigitalInput(id);
                aliases.put(name, IO);
            }
        }
    }

    //!ranges
    public double GetDistance(int id){
        double distance = 0;

        if ("CANrange".equals(RangeType)) {
            CANrange m_CANRange = new CANrange(id);
            distance = m_CANRange.getDistance().getValue().in(Meter);
            m_CANRange.close();
        }
        return distance;
    }

    public double GetDistance(String name, boolean close) {
        Object range = aliases.get(name);
        double value = 0;

        if (range instanceof CANrange) {
            CANrange m_range = (CANrange) range;
            value = m_range.getDistance().getValue().in(Meter);
            if (close == true) {
                m_range.close();
            }
        }
        return value;
    }

    public void setRangeName(int id, String name) {
        if (aliases.get(name) == null) {
            if ("CANrange".equals(RangeType)) {
                CANrange range = new CANrange(id);
                aliases.put(name, range);
            }
        }
    }

    //!utils
    public void canBusMetrics() {

    }

    public String MotorType(Object obj) {
        MotorType_ = obj.getClass().toString();
        return obj.getClass().toString();
    }

    public String RangeType(Object obj) {
        RangeType = obj.getClass().toString();
        return obj.getClass().toString();
    }

    public String InputType(Object obj) {
        InputType = obj.getClass().toString();
        return obj.getClass().toString();
    }

    //!math
    public double PI() {
        double pi = 3.14159265358979323846264338327950288;
        return pi;
    }

    public double E() {
        double e = 2.71828182845904523536028747135266249;
        return e;
    }

    public double toRadians(double degrees) {
        double radians = (degrees * (PI()/180));
        return radians;
    }

    public double toDegrees(double radians) {
        double degrees = (radians * (180/PI()));
        return degrees;
    }
}
