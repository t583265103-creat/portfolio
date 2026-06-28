#include <Eigen/Dense>

class RansacEstimator :public MotionEstimator {
    RansacEstimator(double currentTime);
public:
    RansacEstimator(double currentTime);

private:
    vector<Point2f> creatrePtsFrameNext(const std::vector<TrackedObject>& best_objs);//פונקציה שמקבלת את רשימת האוביקטים הטובים ביותר ובונה מהם בסיס לפריים הבא
    vector<Point2f> creatrePtsFrameBack(const std::vector<TrackedObject>& best_objs);//פונקציה שבונה מפה מהאוביקטים העכשווים על פי מהאוביקטים שהיו הטובים ביותר פריים שעבר 
    virtual Eigen::MatrixXd build_t() override;
    

    Eigen::Matrix3d current_pose; // המיקום הגלובלי המצטבר (X, Y, Theta)
    Eigen::Matrix3d delta_T_camera;

}