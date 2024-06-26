// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Northstar extends SubsystemBase {
    private NetworkTable northstar;
    private DoubleArraySubscriber observations;

    /** Creates a new ExampleSubsystem. */
    public Northstar() {
        this.northstar = NetworkTableInstance.getDefault().getTable("/dev/video0/output/");

        this.observations = this.northstar.getDoubleArrayTopic("observations").subscribe(new double[] {});
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (this.observations.exists()) {
            var q = this.observations.readQueue();

        }
    }
}
