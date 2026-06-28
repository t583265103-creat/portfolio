#pragma once
#include "SensorObject.hpp"
#include "Constants.hpp"
#include <Eigen/Dense>

struct EncoderObject : public SensorObject {
    // --- נתוני מדידה ישירים מהחיישן ---
    double left_velocity;     // מהירות גלגל שמאל (rad/s)
    double right_velocity;    // מהירות גלגל ימין (rad/s)
    double v_linear;          // מהירות קווית משולבת (m/s)
    double v_angular;         // מהירות זוויתית (rad/s)  
    // בנאי המגדיר וקטור מדידה בגודל 6 (כפי שמופיע ב-Z)
    EncoderObject() : SensorObject(Config::DimWheelchairStateVector, Config::MeasurementSizeEncoder, SensorType::Encoder ,false,false,false),
                      left_velocity(0.0), 
                      right_velocity(0.0), 
                      v_linear(0.0), 
                      v_angular(0.0)
                  {}
};