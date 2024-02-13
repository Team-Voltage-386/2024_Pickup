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
    //Motors for running the ground pickup mechanisms
    private static CANSparkMax m_frontPickup;
    private static CANSparkMax m_rearPickup;

    //Tab to hold information on the pickup's states
    private static ShuffleboardTab PickupTab;

    //DIO to detect if the robot has picked up and is holding a note
    private static DigitalInput holdingPieceDetector;

    //Pneumatics to lower the pickup mechanisms to the ground
    private static DoubleSolenoid IntakePneumatics;
    //Pneumatics to lift a loaded note into the shooter
    private static DoubleSolenoid LoadPneumatics;

    //Flag to say if the pickups are deployed and running
    private static boolean pickupEnabled = false;
    //Flag to say if there is a note loaded in the shooter the shooter will have to set this to false once it launches a piece
    public static boolean noteLoaded = false;

    //Temporarry widgets
    private SimpleWidget tempHoldingPieceTF;
    private SimpleWidget tempNoteLoadedTF;

    //Usefull info widgets

    public Pickup() 
    {
        //Instanciates the pickup motors
        m_frontPickup = new CANSparkMax(Constants.kFrontPickupID, MotorType.kBrushless);
        m_rearPickup = new CANSparkMax(Constants.kRearPickupID, MotorType.kBrushless);
        //Remove true to uninvert follower motor
        m_rearPickup.follow(m_frontPickup, true);
        
        //Instanciates the piece detector
        holdingPieceDetector = new DigitalInput(Constants.kPieceDetector);

        PickupTab = Shuffleboard.getTab("Pickup");

        tempHoldingPieceTF = PickupTab.add("Holding Piece Sensor", false);
        tempNoteLoadedTF = PickupTab.add("Loaded Sensor", false);

        //Instanciates the pickup pneumatics and retracts the pickups into the frame
        IntakePneumatics = new DoubleSolenoid(Constants.kPneumaticsModule, PneumaticsModuleType.CTREPCM, 0, 0);
        IntakePneumatics.set(Value.kReverse);

        //Instanciates the load pneumatics and puts it in pickup mode
        LoadPneumatics = new DoubleSolenoid(Constants.kPneumaticsModule, PneumaticsModuleType.CTREPCM, 0, 0);
        LoadPneumatics.set(Value.kReverse);
    }

    public static void runPickup()
    {
        if (!pickupEnabled)
        {
            if (!noteLoaded && holdingPieceDetector.get())
            {
                pickupEnabled = true;
                IntakePneumatics.set(Value.kForward);
                //Change to ideal voltage
                m_frontPickup.setVoltage(4);
            }
        } 
        else 
        {
            pickupEnabled = false;
            retractPickup();
        }
    }

    public static void retractPickup()
    {
        IntakePneumatics.set(Value.kReverse);
        m_frontPickup.setVoltage(0);
        pickupEnabled = false;
    }

    public static void loadShooter()
    {
        LoadPneumatics.set(Value.kForward);
        noteLoaded = true;
    }

    @Override
    public void periodic() 
    {
        boolean holdingPiece = tempHoldingPieceTF.getEntry().getBoolean(false);
        //Swap to this later "boolean holdingPiece = !holdingPieceDetector.get();""

        //Temporary until
        noteLoaded = tempNoteLoadedTF.getEntry().getBoolean(false);

        if (holdingPiece && pickupEnabled)
        {
            retractPickup();
            loadShooter();
        }

        if (Constants.ControllerConstants.kManipulator.getRawButtonPressed(Constants.ControllerConstants.kY))
        {
            runPickup();
        }
    }
}