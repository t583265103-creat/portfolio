#include <Eigen/Dense>

class MotionEstimator {
public:
  
   virtual ~MotionEstimator() {} // destructor וירטואלי הוא חובה בהורשה
   virtual Eigen::MatrixXd calculateMotion() = 0;
  protected:
    void gradeObjects(std::vector<TrackedObject>& objects);//פונקציה שמנקדת את האוביקט ונותנת לו ציון 
    std::vector<TrackedObject> findBestObjects(const std::vector<TrackedObject>& objects);// פונקציה שמגדירה איזה אוביקטים נכנסים לI CP/ransac

    virtual Eigen::MatrixXd build_t()=0;//בניית המטריצת סיבוב
    void updateVectore(const Eigen::Matrix3d& delta_T, double weight);

 
    Eigen::Matrix3d current_pose; // המיקום הגלובלי המצטבר (X, Y, Theta)
    Eigen::Matrix3d delta_T_lidar;
    Eigen::Matrix3d delta_T_camera;

}








    // std::vector<CameraObject> creatrePtsNew(const std::vector<TrackedObject>& best_objs); //שמירת האוביקטים שמהם תעשה המפה
    // vector<Point2f> creatrePtsNew(const std::vector<TrackedObject>& best_objs);