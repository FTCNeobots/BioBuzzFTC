package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*
* controls:
* Joysticks: movement (field-centric)
* Back: reset yaw
* Dpad: flywheel control
* right shoulders: intake
* left shoulders: drive speed control
* x: transfer off
* y: transfer on
* b: flipper
*
*
*
*
* */


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends OpMode {

    //stores the values of the speeds the motor has to move at in this order: LF, LB, RF, RB
    double[] motorSpeeds = new double[4];
    double maxSpeed = 0.5;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor intake;
    DcMotor transfer;
    Servo flipper;
    Servo turret;
    DcMotorEx flywheelA;
    DcMotorEx flywheelB;
    GoBildaPinpointDriver pinpoint;
    double robotHeading;
    double intakeSpeed = 1;
    double pFlywheel = 30;
    double fFlywheel = 15;


    @Override
    public void init() {
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        flipper = hardwareMap.get(Servo.class, "flipper");
        turret = hardwareMap.get(Servo.class, "turret");

        flywheelA = hardwareMap.get(DcMotorEx.class, "fa");
        flywheelB = hardwareMap.get(DcMotorEx.class, "fb");

        flywheelA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelA.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelB.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfFlywheel = new PIDFCoefficients(pFlywheel, 0, 0, fFlywheel);
        flywheelA.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfFlywheel);
        flywheelB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfFlywheel);


    }


    @Override
    public void start(){

        flywheelA.setVelocity(0);
        flywheelB.setVelocity(0);

        flipper.setPosition(downPos);
        turret.setPosition(1);
    }

    @Override
    public void loop() {
        pinpoint.update();
        robotHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        telemetry.addData("Current target velocity: ", currentTargetVelocity * TPStoRPM);
        telemetry.addData("Current velocity A: ", flywheelA.getVelocity() * TPStoRPM);
        telemetry.addData("Current velocity B: ", flywheelB.getVelocity() * TPStoRPM);
        telemetry.update();

        DriveTrain();
        IntakeControl();
        FlywheelControl();
        TransferControl();
        FlipperControl();
    }

    void DriveTrain(){

        if(gamepad1.left_trigger > 0){
            maxSpeed = 1;
        }else if(gamepad1.left_bumper){
            maxSpeed = 0.3;
        }else{
            maxSpeed = 0.5;
        }

        if(gamepad1.back){
            pinpoint.resetPosAndIMU();
        }

        motorSpeeds = MDT.Mecanum(maxSpeed, gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, robotHeading);
        leftFront.setPower(motorSpeeds[0]);leftBack.setPower(motorSpeeds[1]);rightFront.setPower(motorSpeeds[2]);rightBack.setPower(motorSpeeds[3]);

    }

    ///Allows intake to spin both ways using right bumper and trigger
    void IntakeControl(){
        if(gamepad1.right_bumper){
            intake.setPower(intakeSpeed);
        }else if(gamepad1.right_trigger > 0){
            intake.setPower(-intakeSpeed);
        }else{
            intake.setPower(0);
        }
    }


    double RPMtoTPS = (28.0/60.0) * 3/4;
    double TPStoRPM = (60.0/28.0) * 4/3;
    double highVelocity = 3000 * RPMtoTPS;
    double lowVelocity = 1500 * RPMtoTPS;
    double currentTargetVelocity = highVelocity;




    ///Allows flywheels to spin in desired direction, and turn off, using dpad up and down
    void FlywheelControl(){

        if(gamepad1.dpad_up){
            flywheelA.setVelocity(currentTargetVelocity);
            flywheelB.setVelocity(currentTargetVelocity);
        }
        if(gamepad1.dpad_down){
            flywheelA.setVelocity(0);
            flywheelB.setVelocity(0);
        }

        if(gamepad1.dpad_left){
            currentTargetVelocity = lowVelocity;
            flywheelA.setVelocity(currentTargetVelocity);
            flywheelB.setVelocity(currentTargetVelocity);
        }

        if(gamepad1.dpad_right){
            currentTargetVelocity = highVelocity;
            flywheelA.setVelocity(currentTargetVelocity);
            flywheelB.setVelocity(currentTargetVelocity);
        }

    }

    void TransferControl(){

        if(gamepad1.y){
            transfer.setPower(1);
        }
        if(gamepad1.x){
            transfer.setPower(0);
        }

    }


    double currentTime;
    double upPos = 0.8;
    double downPos = 0.5;
    double cooldownTime = 0.5;
    int flipperState = 1;

    void FlipperControl(){
        switch (flipperState){
            //down position, waiting for input
            case 1:
                flipper.setPosition(downPos);
                if(gamepad1.b){
                    currentTime = getRuntime();
                    flipper.setPosition(upPos);
                    flipperState = 2;
                }
                break;
            //moving up
            case 2:
                if((getRuntime()-currentTime) > cooldownTime){
                    currentTime = getRuntime();
                    flipper.setPosition(downPos);
                    flipperState = 3;
                }
                break;
            //moving down
            case 3:
                if((getRuntime()-currentTime) > cooldownTime){
                    flipperState = 1;
                }
                break;
        }
    }
}