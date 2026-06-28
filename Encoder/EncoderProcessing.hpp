#pragma once
#include <vector>
#include "EncoderObject.hpp"
#include "Thread\SensorProducerManager.hpp"
#include "SensorProcessing.hpp"

// תיקון השם ל-EncoderProcessing
class EncoderProcessing : public SensorProcessing<EncoderMeasurement, EncoderObject> {
public:
    EncoderProcessing() = default; 
    // חתימה שתואמת בדיוק ל-Base Class
    EncoderObject process(EncoderMeasurement& input, double ts) override;

private:
    EncoderObject createObject(EncoderMeasurement& raw);
};