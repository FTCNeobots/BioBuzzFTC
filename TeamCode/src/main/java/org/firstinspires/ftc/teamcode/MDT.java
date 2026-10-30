package org.firstinspires.ftc.teamcode;

public class MDT {


    ///returns the values the motors have to move at, in this order: LF, LB, RF, RB
    /// @param _maxSpeed the max allowed speed
    /// @param _lStickY left stick y value
    /// @param _lStickX left stick x value
    /// @param _rStickX right stick x value
    /// @param _botHeading the heading of the robot
    /// @return an array with the drive train motor speeds in the order: LF, LB, RF, RB
    public static double[] Mecanum(double _maxSpeed, double _lStickX, double _lStickY, double _rStickX, double _botHeading){

        double lf, lb, rf, rb;
        double[] allSpeeds = new double[4];

        double _X = _lStickX * Math.cos(_botHeading) - _lStickY * Math.sin(_botHeading);
        double _Y = _lStickX * Math.sin(_botHeading) + _lStickY * Math.cos(_botHeading);

        _rStickX *= 1.1;

        lf = MathLogic.Clamp(_Y - _X - _rStickX, -1, 1) * _maxSpeed;
        lb = MathLogic.Clamp(_Y + _X - _rStickX, -1, 1) * _maxSpeed;
        rf = MathLogic.Clamp(_Y + _X + _rStickX, -1 ,1) * _maxSpeed;
        rb = MathLogic.Clamp(_Y - _X + _rStickX, -1, 1) * _maxSpeed;

        allSpeeds[0] = lf;
        allSpeeds[1] = lb;
        allSpeeds[2] = rf;
        allSpeeds[3] = rb;

        return allSpeeds;

    }

}
