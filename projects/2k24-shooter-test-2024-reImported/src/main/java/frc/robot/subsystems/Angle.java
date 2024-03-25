package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;

public class Angle extends SubsystemBase {

    private CANSparkMax m_motor;
    private static final int angleID = 41;
    

    private RelativeEncoder m_relEncoder;
    private AbsoluteEncoder m_absEncoder;


    public Angle() {
        SetupAngle();
    }

    public void SetupAngle() {

        SmartDashboard.putNumber("Realative Angle", 0);
        SmartDashboard.putNumber("Angle Target", 0);

        m_motor = new CANSparkMax(angleID, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();
        m_relEncoder = m_motor.getEncoder(); // (SparkRelativeEncoder.Type.kHallSensor, 42);
        //m_absEncoder = m_motor.getAbsoluteEncoder();

        m_relEncoder.setPosition(0);

        m_motor.setIdleMode(IdleMode.kCoast);

        SmartDashboard.putNumber("point error", 0);

    }

    
    @Override
    public void periodic() {
        var fowardSwitch = m_motor.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);

        SmartDashboard.putBoolean("Foward Switch", fowardSwitch.isPressed());
    }


    /* 

    public void setAngle() {

        double relativeEncoderPosition = m_relEncoder.getPosition();
        SmartDashboard.putNumber("Relative Angle", relativeEncoderPosition);
        
        //double absoluteEncoderPosition = m_absEncoder.getPosition();
        //SmartDashboard.putNumber("Absolute Angle", absoluteEncoderPosition);

        double targetAngle = SmartDashboard.getNumber("Angle Target", 0);
        //m_motor.set(targetAngle);

    }

    */

    public void setAnglePID() {
        
        double relativeEncoderPosition = m_relEncoder.getPosition();
        SmartDashboard.putNumber("Relative Angle", relativeEncoderPosition);

        var m_pidController = m_motor.getPIDController();

        double kP = 0.1; 
        double kI = 1e-5;
        double kD = 0.1;


        m_pidController.setP(kP);
        m_pidController.setI(kI);
        //m_pidController.setD(kD);

        double targetRotation = SmartDashboard.getNumber("Angle Target", 0);
        double calculatedRotations = targetRotation * (57.2 / (58.2 + 2));

        m_pidController.setReference(calculatedRotations, CANSparkMax.ControlType.kPosition);

        double pointErr = relativeEncoderPosition - targetRotation;

        SmartDashboard.putNumber("point error", pointErr);

    }
    
}
