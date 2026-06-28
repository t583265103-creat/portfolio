
#include "EncoderProcessing.hpp"
#include "EncoderData.hpp"
#include "EncoderObject.hpp"
#include "SensorObject.hpp"
#include <algorithm>
#include <cmath>

EncoderObject EncoderProcessing::process(EncoderMeasurement& input , double tss)
{
    this->ts=tss;
    EncoderObject EO = createObject(input);  
    return EO;
}



// EncoderMeasurement EncoderProcessing::clean(std::vector<EncoderMeasurement>& EM) {
//     // לוגיקת סינון. למשל, סינון קפיצות חריגות בטיקים של האנקודר
//     std::vector<EncoderMeasurement> cleaned;
//     for(auto& m : EM) {
//         if(m.raw_left_ticks >= 0 && m.raw_right_ticks >= 0) { // סינון בסיסי לדוגמה
//             cleaned.push_back(m);
//         }
//     }
//     return cleaned;
// }

EncoderObject EncoderProcessing::createObject(EncoderMeasurement& raw) {
    EncoderObject EO;   
    
    double WHEEL_RADIUS = Config::WHEEL_RADIUS;
    double TRACK_WIDTH = Config::TRACK_WIDTH;
    double TICKS_TO_RAD = Config::TICKS_TO_RAD;
    
    double dt = Config::EncedorDt; 
    if (dt <= 0.0) {
        // אם משום מה ה-dt הוא 0, נמנע קריסה
        return EO; 
    }

    EO.timestamp = this->ts; // (בהנחה שיש לך שדה כזה ב-raw)

    EO.left_velocity = (raw.left_ticks * TICKS_TO_RAD / dt) * WHEEL_RADIUS;
    EO.right_velocity = (raw.right_ticks * TICKS_TO_RAD / dt) * WHEEL_RADIUS;

    EO.v_linear = (EO.right_velocity + EO.left_velocity) / 2.0;

    EO.v_angular = (EO.right_velocity - EO.left_velocity) / TRACK_WIDTH;

    // אמינות: Encoder הוא חיישן אמין יחסית (לא היה מחושב כלל קודם — נשאר 0.0,
    // מה שגרם ל-noise_scale=(1/conf)^2 ב-EncoderData::buildR להתפוצץ ל-עשרות
    // אלפים, ולכן ה-Kalman gain על Vx/Vyaw היה כמעט אפס). מורידים אותה רק
    // במהירות לא סבירה (ייתכן רעש/slip), אחרת אמינות גבוהה כברירת מחדל.
    double speed_for_conf = std::abs(EO.v_linear);
    EO.confidence = std::clamp(1.0 - (speed_for_conf / 5.0), 0.3, 1.0);

    EncoderData ED;
    ED.process(EO);
    
    return EO;
    
}
