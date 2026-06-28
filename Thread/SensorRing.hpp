#pragma once
#include <mutex>
#include <stdexcept>
#include <condition_variable>
#include "SensorMeasurement.hpp"
#include <vector>
#include <memory>
template <typename T, int SIZE = 10>
class SensorRing {
protected: 
    SensorMeasurement<T> data[SIZE]; 
    int head = 0;        
    int count = 0; 
    mutable std::mutex mtx;
    std::condition_variable cv; 
public:
    virtual ~SensorRing() = default;

    virtual void addData(const SensorMeasurement<T>& value) {
        std::lock_guard<std::mutex> lock(mtx);
        data[head] = value;
        head = (head + 1) % SIZE; 
        
        if (count < SIZE) {
            count++;
        }
        cv.notify_one(); // <--- "צלצול בפעמון": מעיר תהליכון אחד שמחכה
    }

// פונקציה חוסמת: מחכה עד שיש N מדידות בבאפר, ואז שולפת אותן
   virtual std::vector<SensorMeasurement<T>> popLastNBlocking(int n) {
        std::unique_lock<std::mutex> lock(this->mtx); // חייבים unique_lock עבור ה-cv
        
        // התנאי: מחכה כאן עד ש-count יהיה גדול או שווה ל-N
        // אם לא הגיעו מספיק נתונים, התהליכון "נרדם" (לא צורך מעבד!)
        cv.wait(lock, [this, n] { return this->count >= n; });

        // ברגע שהגענו לכאן, אנחנו בטוחים שיש מספיק נתונים
        std::vector<SensorMeasurement<T>> results(n);
        
        for (int i = 0; i < n; ++i) {
            int index = (this->head - n + i + SIZE) % SIZE;
            results[i] = this->data[index];
        }

        this->count -= n;
        this->head = (this->head - n + SIZE) % SIZE;

        return results;
    }

    // פונקציה לא-חוסמת עם timeout: מחכה עד ל-N מדידות או עד שמתקבל timeout
    virtual std::vector<SensorMeasurement<T>> popLastNWithTimeout(int n, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(this->mtx);

        bool ok = cv.wait_for(lock, timeout, [this, n] { return this->count >= n; });

        if (!ok) {
            return {};
        }

        std::vector<SensorMeasurement<T>> results(n);
        for (int i = 0; i < n; ++i) {
            int index = (this->head - n + i + SIZE) % SIZE;
            results[i] = this->data[index];
        }

        this->count -= n;
        this->head = (this->head - n + SIZE) % SIZE;

        return results;
    }
};