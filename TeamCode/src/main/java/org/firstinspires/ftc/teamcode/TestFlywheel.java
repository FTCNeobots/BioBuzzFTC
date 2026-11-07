package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name = "TestFlywheel")
public class TestFlywheel extends OpMode {

    DcMotorEx flywheel;
    double RPMtoTPS = 28.0/60.0;
    double TPStoRPM = 60.0/28.0;
    double highVelocity = 5500 * RPMtoTPS * 3/4;
    double lowVelocity = 4000 * RPMtoTPS * 3/4;
    double currentTargetVelocity = highVelocity;
    double P = 0;
    double F = 0;

    double[] stepValues = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;


    @Override
    public void init() {

        flywheel = hardwareMap.get(DcMotorEx.class, "fa");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfFlywheel = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfFlywheel);

    }

    @Override
    public void loop() {

        if(gamepad1.yWasPressed()){
            if(currentTargetVelocity == highVelocity){
                currentTargetVelocity = lowVelocity;
            }else{
                currentTargetVelocity = highVelocity;
            }
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepValues.length;
        }

        if(gamepad1.dpadLeftWasPressed()){
            F -= stepValues[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()){
            F += stepValues[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed()){
            P -= stepValues[stepIndex];
        }
        if(gamepad1.dpadUpWasPressed()){
            P += stepValues[stepIndex];
        }

        PIDFCoefficients pidfFlywheel = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfFlywheel);

        flywheel.setVelocity(currentTargetVelocity);

        //p = 30, f = 11


        double curVelocity = flywheel.getVelocity();
        double error = currentTargetVelocity - curVelocity;

        telemetry.addData("P: ", P);
        telemetry.addData("F: ", F);
        telemetry.addData("Current velocity: ", curVelocity * TPStoRPM);
        telemetry.addData("Current target velocity: ", currentTargetVelocity * TPStoRPM);
        telemetry.addData("Error: ", error * TPStoRPM);
        telemetry.addData("Stepsize: ", stepValues[stepIndex]);
        telemetry.update();

    }
}
