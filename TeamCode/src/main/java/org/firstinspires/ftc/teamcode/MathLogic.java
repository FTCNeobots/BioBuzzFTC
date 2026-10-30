package org.firstinspires.ftc.teamcode;

public class MathLogic {


    ///function to clamp a certain number to stay within a lower and upper bound
    /// @param _input the value you want to clamp
    /// @param _lower the lower bound
    /// @param _upper the upper bound
    /// @return the clamped value as a clamp
    static double Clamp(double _input, double _lower, double _upper){
        if(_input > _upper){
            return _upper;
        } else if (_input < _lower) {
            return _lower;
        }else{
            return _input;
        }
    }
}
