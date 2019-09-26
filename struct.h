#ifndef STRUCT_H
#define STRUCT_H

#include <vector>
#include <mutex>
/*! @brief Stores bogies x and y coordinates into vectors
 */
struct BogieVal{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> rad_friendly;  //radius from friendly
};

/*! @brief Finding the two intercept points
 */
struct IntPoints{
  double x1,x2, y1,y2, range_to_friendly;
};
/*! @brief Point to bogie's x and y coordinates and the range
 * between bogie and friendly
 */
struct BogiePos{
    double x,y,range_friendly;   //range to bogie from Friendly
};
/*! @brief Sets the friendly's linear velocity(speed) and angular velocity (turing speed)
 */
struct Velocities{
    double linearV_,angularV_;
};



#endif // STRUCT_H


