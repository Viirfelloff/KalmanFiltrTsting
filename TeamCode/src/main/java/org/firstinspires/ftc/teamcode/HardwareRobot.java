package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;

public class HardwareRobot {
    public final MotorEx wheel;
    public final MotorEx wheel2;

    public final Limelight3A limelight3A;

    public final GoBildaPinpointDriver pinpoint;

    public HardwareRobot(HardwareMap hardwareMap) {

        ////////////
        // WHEELS //
        ////////////
        wheel = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        wheel2 = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);

        wheel.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        wheel.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wheel.setRunMode(Motor.RunMode.RawPower);
        wheel2.setRunMode(Motor.RunMode.RawPower);


        wheel.setInverted(true);
        wheel2.setInverted(true);

        wheel2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        wheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        wheel2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        /////////////
        // SERVOS  //
        /////////////
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        ////////////
        // CAMERA //
        ////////////
        limelight3A = hardwareMap.get(Limelight3A.class, "Limelight 3A");

    }
}