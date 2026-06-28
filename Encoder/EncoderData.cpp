#include "EncoderData.hpp"
#include "Constants.hpp"
#include "SensorConfig.hpp"
#include <algorithm>

void EncoderData::process(EncoderObject& EO) {
    auto encoder_cfg = ConfigManager::getEncoder("encoder_main");
    Eigen::Vector3f v_linear_sensor(EO.v_linear, 0.0f, 0.0f);
    Eigen::Vector3f v_linear_base = encoder_cfg.transformVector(v_linear_sensor);
    EO.v_linear = v_linear_base.x();
    buildZ(EO);
    buildH(EO);
    buildR(EO);

}

void EncoderData::buildZ(EncoderObject& EO) {
    EO.Z(0) = EO.v_linear;   // המדידה הראשונה: מהירות קווית
    EO.Z(1) = EO.v_angular;
}

void EncoderData::buildH(EncoderObject& EO) {
    EO.H.setZero();
    EO.H(0, static_cast<int>(Config::StateMembersRobot::StateVx)) = 1.0; 
    EO.H(1, static_cast<int>(Config::StateMembersRobot::StateVyaw)) = 1.0;
}

void EncoderData::buildR(EncoderObject& EO) {
    EO.R.setZero();

    // הגנה: confidence חייב בטווח סביר — אחרת noise_scale=(1/conf)^2 מתפוצץ
    // (לדוגמה confidence=0 בטעות -> noise_scale=1,000,000 -> R בעשרות אלפים,
    // וה-Kalman gain על Vx/Vyaw קורס לאפס בפועל).
    float safe_conf = std::clamp(static_cast<float>(EO.confidence), 0.1f, 1.0f);

    // פקטור הרעש מתהפך ביחס לביטחון (ביטחון נמוך = רעש גבוה)
    float inv_conf = 1.0f / safe_conf;
    float noise_scale = inv_conf * inv_conf;

    // הגדרת שונות בסיסית (Variance)
    EO.R(0, 0) = 0.02f * noise_scale;
    EO.R(1, 1) = 0.04f * noise_scale;
}