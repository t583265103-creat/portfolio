// SystemOrchestrator.hpp
#pragma once
#include "SensorWorkerManager.hpp"

class SensorProducer {
public:
    void init();   // רק הצהרה
    void stop();   // רק הצהרה
private:
    SensorWorkerManager worker_manager;
    // פונקציות העזר שהיו הופכות ל-Lambdas בתוך המיין
    void ProducerImu(); 
    void ProducerLidar();
    void ProducerRadar();
    void ProducerCamera();
    void ProducerGps();
    void ProducerEncoder();

};