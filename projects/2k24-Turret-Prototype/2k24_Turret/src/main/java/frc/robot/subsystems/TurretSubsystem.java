// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;

public class TurretSubsystem extends SubsystemBase {

  private static final int TurretMotorDeviceID = 12;

  private CANSparkMax m_turretMotor;

  private SparkPIDController m_pidController;

  private RelativeEncoder m_relEncoder;

  private AbsoluteEncoder m_absEncoder;
  
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {

    /* Dashboard */
    SmartDashboard.putNumber("Turret Target", 0);
    SmartDashboard.putBoolean("use Shuffleboard", false);
    SmartDashboard.putBoolean("reset Enc", false);
    SmartDashboard.putNumber("RPM", 0.0);

    SmartDashboard.putNumber("kP", 0.0);
    SmartDashboard.putNumber("kI", 0.0);
    SmartDashboard.putNumber("kD", 0.0);


    /* Systems */
    m_turretMotor = new CANSparkMax(TurretMotorDeviceID, MotorType.kBrushless);
    m_turretMotor.restoreFactoryDefaults();
    m_turretMotor.setIdleMode(IdleMode.kCoast);

    m_relEncoder = m_turretMotor.getEncoder();
    m_absEncoder = m_turretMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    //m_pidController = setPID(m_turretMotor, 0.001, 0.0, 0.0);
  }

  public SparkPIDController setPID(CANSparkMax m_motor, double kP, double kI, double kD) {

    SparkPIDController pidController = m_motor.getPIDController();

    // double kP = 0.0001; // 0.1
    // double kI = 0.0; // 1e-5
    // double kD = 0.0; // 0.1

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);

    return pidController;

  }

  public void shuffleboardControls() {

    boolean useShuffleboard = SmartDashboard.getBoolean("use Shuffleboard", false);

    boolean resetEncoder = SmartDashboard.getBoolean("reset Enc", false);

    if (resetEncoder) {
      m_relEncoder.setPosition(0);
    }

    double m_rpm = m_relEncoder.getVelocity();
    SmartDashboard.putNumber("RPM", m_rpm);

    double m_relativeMotorEncoderPosition = m_relEncoder.getPosition();
    SmartDashboard.putNumber("rel Encoder", m_relativeMotorEncoderPosition);

    double m_absoluteMotorEncoderPosition = m_absEncoder.getPosition();
    SmartDashboard.putNumber("abs Encoder", m_absoluteMotorEncoderPosition);

    double m_kP = SmartDashboard.getNumber("kP", 0.0);
    double m_kI = SmartDashboard.getNumber("kI", 0.0);
    double m_kD = SmartDashboard.getNumber("kD", 0.0);


    /* Shuffleboard controls */
    if (useShuffleboard) {
      m_pidController = setPID(m_turretMotor, 0.1, 1e-5, 0.1);
      double target = SmartDashboard.getNumber("Turret Target", 0);
      m_pidController.setReference(target, CANSparkMax.ControlType.kPosition);
      double pointError = (target - m_relativeMotorEncoderPosition);
      SmartDashboard.putNumber("Point Err", pointError);
    } else {
      m_pidController.setOutputRange(0, 100);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleboardControls();
  }

  public void setRPM(double targetSpeed) {
    m_pidController.setReference(targetSpeed, CANSparkMax.ControlType.kVelocity);
  }

  public void stop() {
    m_turretMotor.set(0); // try stopMotor()
  }

  public Command setSpeedClockwiseCommand(double m_targetSpeed) {
    //double calculatedRotations = targetRotation * (57.2 / (58.2 + 2));
    return runOnce(
        () -> {
          setRPM(m_targetSpeed);
        });
  }
}
