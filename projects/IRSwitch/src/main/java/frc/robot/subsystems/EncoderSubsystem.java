// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EncoderSubsystem extends SubsystemBase {
  /** Creates a new Encoder. */

  public static final int[] ENC_DIO_FRONT_RIGHT = {4,5};

  public static final double WHEEL_DIAMETER = 4.0;
  public static final double PULSE_PER_REVOLUTION = 250;  //Need to revisit this value!!
  public final double DISTANCE_PER_PULSE = (double)(Math.PI*WHEEL_DIAMETER)/PULSE_PER_REVOLUTION;

  private Encoder _enc_leftRear = new Encoder(ENC_DIO_FRONT_RIGHT[0], ENC_DIO_FRONT_RIGHT[1]);
  public static final String MYNAME = "DriveTrain";

  public EncoderSubsystem() {
    SetupEncoder(_enc_leftRear, "LEFTFRONT", true);
  }

  private void SetupEncoder(Encoder enc, String name, boolean reverseDirection) {
    
    if (enc == null)
       return;
    enc.setMaxPeriod(.1);
    enc.setMinRate(10);
    System.out.println(MYNAME + ": Distance per Pulse: " + DISTANCE_PER_PULSE);
    enc.setDistancePerPulse(DISTANCE_PER_PULSE);
    enc.setReverseDirection(reverseDirection);
    enc.setSamplesToAverage(7);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder", _enc_leftRear.get());
  }
}
