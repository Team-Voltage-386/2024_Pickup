package frc.robot.subsystems;

import frc.robot.Constants;
import frc.utils.Flags.subsystemsStates;
import frc.utils.Flags;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pickup extends SubsystemBase {
    //Motors for running the ground pickup mechanisms
    private static CANSparkMax m_frontPickup;
    private static CANSparkMax m_rearPickup;

    //Motor on the shooter that loads the piece into the shooter
    private static TalonSRX loaderMotor;

    //Tab to hold information on the pickup's states
    private static ShuffleboardTab PickupTab;

    //DIO to detect if the robot has picked up and is holding a note
    private static DigitalInput loadedPieceDetector;
    private static DigitalInput secondaryPieceDetector;

    //Pneumatics to lower the pickup mechanisms to the ground
    private static DoubleSolenoid IntakePneumatics;
    //Pneumatics to lift a loaded note into the shooter
    private static DoubleSolenoid LoadPneumatics;
    //Pneumatics to latcht the loader and lock it in place or realese it from the shooter
    private static DoubleSolenoid LatchPneumatics;

    //True if the pickups are deployed and running
    private static boolean pickupEnabled = false;
    //True if the piece loader is actuated up
    private static boolean loaderEnabled = false;

    //Flag to say the current state of the piece
    public static subsystemsStates noteState;

    //Temporarry widgets
    private SimpleWidget tempHoldingPieceTF;

    //Usefull info widgets

    public Pickup() 
    {
        //Instanciates the pickup motors
        m_frontPickup = new CANSparkMax(Constants.kFrontPickupID, MotorType.kBrushless);
        m_rearPickup = new CANSparkMax(Constants.kRearPickupID, MotorType.kBrushless);

        //Instanciates the loader motor
        loaderMotor = new TalonSRX(Constants.kLoaderMotorID);

        //Remove true to uninvert follower motor
        m_rearPickup.follow(m_frontPickup, true);
        
        //Instanciates the piece detectors
        loadedPieceDetector = new DigitalInput(Constants.kLoadedPieceDetectorChannel);
        secondaryPieceDetector = new DigitalInput(Constants.kSecondaryPieceDetectorChannel);

        PickupTab = Shuffleboard.getTab("Pickup");

        tempHoldingPieceTF = PickupTab.add("Holding Piece Sensor", false);

        //Instanciates the pickup pneumatics and retracts the pickups into the frame
        IntakePneumatics = new DoubleSolenoid(Constants.kPneumaticsModule, PneumaticsModuleType.CTREPCM, 0, 0);
        IntakePneumatics.set(Value.kReverse);

        //Instanciates the load pneumatics and puts it in pickup mode
        LoadPneumatics = new DoubleSolenoid(Constants.kPneumaticsModule, PneumaticsModuleType.CTREPCM, 0, 0);
        LoadPneumatics.set(Value.kReverse);

        //Instanciates the latch pneumatics and retracts it in to unlcoked mode
        LatchPneumatics = new DoubleSolenoid(Constants.kPneumaticsModule, PneumaticsModuleType.CTREPCM, 0, 0);
        LatchPneumatics.set(Value.kReverse);

        //Starts off with no piece (will automaitically change if it sees a piece is loaded)
        noteState = subsystemsStates.noPiece;
    }

    //Called to prematurely stop the pickup or start it running
    public static void runPickup()
    {
        if (!pickupEnabled)
        {
            if (noteState==subsystemsStates.noPiece)
            {
                pickupEnabled = true;
                IntakePneumatics.set(Value.kForward);
                //Change to ideal running voltage
                m_frontPickup.setVoltage(4);
            }
        } 
        else 
        {
            pickupEnabled = false;
            retractPickup();
        }
    }

    //Called when we want to bring back the pickup mechanism into the frame
    public static void retractPickup()
    {
        IntakePneumatics.set(Value.kReverse);
        //Set to ideal resting voltage
        m_frontPickup.setVoltage(0);
        pickupEnabled = false;
    }

    //Runs to bring the loade piece up to the shooter
    public static void loadShooter()
    {
        //Adjust to ideal percentage
        loaderMotor.set(TalonSRXControlMode.PercentOutput, 0.5);
        LoadPneumatics.set(Value.kForward);
        loaderEnabled = true;
    }

    //Call to lock the 
    public static void depressurizeLoader()
    {
        LoadPneumatics.set(Value.kOff);
    }

    //Locks the loader in place
    public static void lockLoader()
    {
        LatchPneumatics.set(Value.kForward);
    }

    //Call after a piece is shot to lower the lift into the next mode & unlocks loader
    public static void unloadShooter()
    {
        loaderMotor.set(TalonSRXControlMode.PercentOutput, 0);
        LatchPneumatics.set(Value.kReverse);
        loaderEnabled = false;
        noteState = subsystemsStates.noPiece;
    }

    @Override
    public void periodic() 
    {
        //Swap to this later "if (noteState==subsystemsStates.noPiece && (!loadedPieceDetector.get() || !secondaryPieceDetector.get()))""
        if (tempHoldingPieceTF.getEntry().getBoolean(false))
        {
            noteState = subsystemsStates.holdingPiece;
        }
        //Remove entire else statement later
        else
        {
            noteState = subsystemsStates.noPiece;
        }

        //The piece is loaded when ONLY the loadedPieceDectector sensor sees a piece and the loader is enabled
        if (loaderEnabled && !loadedPieceDetector.get() && secondaryPieceDetector.get())
        {
           noteState=subsystemsStates.loadedPiece;
           loaderMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }


        if (noteState==subsystemsStates.holdingPiece && pickupEnabled)
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