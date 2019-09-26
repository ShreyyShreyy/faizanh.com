/*! @file
 *
 *  @brief Main entry point for assignment 3.
 *
 *  41012 Programming For Mechatronica Systems Autumn 2019
 *
 *  @author {Shreynit Prasad 12602520}
 *  @date {27/09/19}
*/
#ifndef PurePursuit_H
#define PurePursuit_H

#include <condition_variable>
#include <mutex>

#include "math.h"
#include "simulator.h"
//#include "PurePursuit.h"
#include "struct.h"

class PurePursuit
{
public:

    PurePursuit(const std::shared_ptr<Simulator>&sim);
    /*! @brief locates the distance between bogie to friendly and bogie to base by using mutex and conditional variable.
     * Then proceeds to  calculate linear Extrapolate and two IntPoints for storing
     * The results are then used in FriendlyControl
     */
    void Watchdog();
    /*! @brief Seperate the threads coming in and updating the range of the bogie to friendly
     * through mutex
     */
    void DistanceFromBase();
    /*! @brief Seperate the threads coming in and updating the range of the bogie to base
     * through mutex
     */
    void DistanceFromFriendly();
    void findBogie();
    /*! @brief Controls friendly by initalising parameters
     * collect data from findEnemy
     * Locate which quadrant the bogie is
     * Decide which direction to turn to meet the bogie the quickest way
     * Find the difference in orienation
     * Travel at the correct velocity
     */
    void FriendlyControl();
    /*! @brief reads and excute velocity values
     */



private:
    /*! @brief Mutex to retrieve bogie positions
     */
    void setBogieToBase(RangeStamped bogie_range_from_base);
    /*! @brief Mutex to retrieve bogie positions
     */
    void setBogieToFriendly(RangeStamped bogie_range_from_friendly);

    /*! @brief Get Friendly Position from Simulator
     */
    Pose getFriendlyPose();
    /*! @brief Get bogie Position from Simulator
     */
    BogiePos getBogiePos();
    /*! @brief Get Velocity values
     */
    Velocities getVelocities();
    void setFriendlyPose(Pose);
    void setBogiePos(BogiePos);
    void setVelocities(Velocities);
    /*! @brief Linear Extrapolate to find the bogie range
     */

    RangeStamped Extrapolate(std::vector<RangeStamped> bogie_x_coord, double Timestamp_);
    /*! @brief Used to calculate and find the two intercept points from bogie to base and bogie to friend
     */
    IntPoints interceptCalc(double range_to_bogie, double range_to_friendly, double friendly_xpose,double friendly_ypose);
    /*! @brief Using interceptCalc. we can use the points to find the shortest bogie to friendly range
     * and travel to the correct interception
     */
    BogiePos FindPath(double* path, IntPoints poi, BogieVal* bogie_position);
    /*! @brief Pointing to Simulator Header
     */
    std::shared_ptr<Simulator> simulation_;
    //--vectors---//
    /*! @brief container to store bogie and friendly range
     */
    std::vector<RangeStamped> bogie_to_friendly_;
    //--bools----//
    /*! @brief used to notify these variable
     */
    std::atomic<bool> bogie_position;
    std::atomic<bool> friendly_to_bogie_R_;
    /*! @brief used to notify these variable
     */
    std::atomic<bool> base_to_bogie_R_;
    /*! @brief used to notify these variable
     */
    //--callouts----//
    /*! @brief Send Velocity to sim
     */
    Velocities required_velocities_;
    /*! @brief Send Friend cord and orienation to sim
     */
    Pose friendly_pose_;
    /*! @brief Send Bogie cord and orienation to sim
     */
    BogiePos bogie_position_;
    RangeStamped bogie_range_from_base_;
    //---mutex-----//
    std::mutex mutex_bogie_data_;
    std::mutex mutex_required_velocities_;
    std::mutex mutex_bogie_position_;
    std::condition_variable range_recieved_;
    std::condition_variable convar_bogie_position_;

};

#endif // PurePursuit_H
