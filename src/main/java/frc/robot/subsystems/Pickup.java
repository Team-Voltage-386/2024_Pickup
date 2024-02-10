package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pickup extends SubsystemBase {
    private static CANSparkMax m_frontPickup;
    private static  CANSparkMax m_rearPickup;

    private static ShuffleboardTab PickupTab;

    private static DigitalInput holdingPieceDetector;

    private static DoubleSolenoid IntakePneumatics;

    private static boolean pickupEnabled = false;

    public Pickup() 
    {
        m_frontPickup = new CANSparkMax(Constants.kFrontPickupID, MotorType.kBrushless);
        m_rearPickup = new CANSparkMax(Constants.kRearPickupID, MotorType.kBrushless);
        //Remove true to uninvert follower motor
        m_rearPickup.follow(m_frontPickup, true);
        
        holdingPieceDetector = new DigitalInput(Constants.kPieceDetector);

        PickupTab = Shuffleboard.getTab("Pickup");

        IntakePneumatics = new DoubleSolenoid(Constants.kPneumaticsModule, PneumaticsModuleType.CTREPCM, 0, 0);
        IntakePneumatics.set(Value.kReverse);
    }

    public static void runPickup()
    {
        if (!pickupEnabled && holdingPieceDetector.get())
        {
            pickupEnabled = true;
            IntakePneumatics.set(Value.kForward);
            //Change to ideal voltage
            m_frontPickup.setVoltage(4);
        }
    }


    @Override
    public void periodic() 
    {
        if (!holdingPieceDetector.get() && pickupEnabled)
        {
            pickupEnabled = false;
            IntakePneumatics.set(Value.kReverse);
            m_frontPickup.setVoltage(0);
        }

        if (Constants.ControllerConstants.kManipulator.getRawButtonPressed(Constants.ControllerConstants.kY))
        {
            runPickup();
        }
    }
}