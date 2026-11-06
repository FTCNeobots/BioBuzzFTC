package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "FirstClass")
public class FirstClass extends OpMode {

    //stores the values of the speeds the motor has to move at in this order: LF, LB, RF, RB
    double[] motorSpeeds = new double[4];
    double maxSpeed = 1;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor intake;
    DcMotorEx flywheelA;
    DcMotorEx flywheelB;
    GoBildaPinpointDriver pinpoint;
    double robotHeading;
    double intakeSpeed = 1;
    double pFlywheel = 0;
    double fFlywheel = 0;


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
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

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



    }

    @Override
    public void loop() {
        pinpoint.update();
        robotHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        telemetry.addData("Heading = ", robotHeading);

        if(gamepad1.back){
            pinpoint.resetPosAndIMU();
        }


        motorSpeeds = MDT.Mecanum(maxSpeed, gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, robotHeading);
        leftFront.setPower(motorSpeeds[0]);leftBack.setPower(motorSpeeds[1]);rightFront.setPower(motorSpeeds[2]);rightBack.setPower(motorSpeeds[3]);

        IntakeControl();
        FlywheelControl();
    }

    ///Allows intake to spin both ways using right bumper and trigger
    void IntakeControl(){
        if(gamepad1.right_bumper){
            intake.setPower(intakeSpeed);
        }
        if(gamepad1.right_trigger > 0){
            intake.setPower(-intakeSpeed);
        }
    }


    double RPMtoTPS = 28.0/60.0;
    double TPStoRPM = 60.0/28.0;
    double highVelocity = 5500 * RPMtoTPS;
    double lowVelocity = 4000 * RPMtoTPS;
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

    }


    //TO DO:
    //Flywheel can shoot, different RPMs



}