#include "PurePursuit.h"

#include <iostream>

using namespace std;


PurePursuit::PurePursuit(const std::shared_ptr<Simulator> &sim)
{
    simulation_=sim;

}

//----------------------Mutex and ConVar------------------------------------//
                   /*Mutexs with Setters and Getters*/

//------------------------Setters------------------------------------------//
void PurePursuit::setBogieToFriendly(RangeStamped bogie_range_from_friendly){
    mutex_bogie_data_.lock();
    bogie_to_friendly_.insert(bogie_to_friendly_.begin(), bogie_range_from_friendly);
    if(bogie_to_friendly_.size() > 8){                                                      //Checks 2 of the data which is compared at a single time
        bogie_to_friendly_.pop_back();                                                      // Ensures 2 Vectors are stored
    }else if(bogie_to_friendly_.size()<8){
        bogie_to_friendly_.insert(bogie_to_friendly_.begin(), bogie_range_from_friendly);   //Gets the information from the vector.
    }
    mutex_bogie_data_.unlock();
}

void PurePursuit::setFriendlyPose(Pose pose){
    mutex_bogie_data_.lock();
    friendly_pose_=pose;
    mutex_bogie_data_.unlock();
}

void PurePursuit::setVelocities(Velocities velocity_val){
    mutex_required_velocities_.lock();
    required_velocities_=velocity_val;
    mutex_required_velocities_.unlock();
}


void PurePursuit::setBogieToBase(RangeStamped bogie_range_from_base){
    mutex_bogie_data_.lock();
    bogie_range_from_base_=bogie_range_from_base;
    mutex_bogie_data_.unlock();
}



void PurePursuit::setBogiePos(BogiePos bogie_stat){
    mutex_bogie_position_.lock();
    bogie_position_ = bogie_stat;
    mutex_bogie_position_.unlock();
    convar_bogie_position_.notify_all();
    bogie_position = true;
}

//------------------GETTERS----------------------------------//
Velocities PurePursuit::getVelocities(){
    std::unique_lock<std::mutex> lock(mutex_required_velocities_);
    return required_velocities_;
}
Pose PurePursuit::getFriendlyPose(){
    std::unique_lock<std::mutex> lock(mutex_bogie_data_);
    return friendly_pose_;
}
BogiePos PurePursuit::getBogiePos(){
    std::unique_lock<std::mutex> lock(mutex_bogie_position_);
    bogie_position=false;
    return bogie_position_;
}



BogiePos PurePursuit::FindPath(double* path, IntPoints poi, BogieVal* position_of_bogie){
    BogiePos return_position_of_bogie;                                                            // after picking the correct path, the vaules of the intercept coordinates are stored into StoreBogieVaules
    int intercept;
    if(position_of_bogie->x.size()>0){
        double intercept1 =pow((pow(poi.x1-position_of_bogie->x.front(), 2)+pow(poi.y1-position_of_bogie->y.front(),2)),0.5);      // taking x and y coordinates of the intercepting point calculated in interceptCalc

        double intercept2 =pow((pow(poi.x2-position_of_bogie->x.front(), 2)+pow(poi.y2-position_of_bogie->y.front(),2)),0.5);        // and comparing it against each other.


        if (intercept2>intercept1){                                                              //ideally, the bogie would most likely to be at the smallest Bogie to Friendly range
            intercept=1;
        }
        else{
            intercept=2;
        }

        if (intercept == 1)
        {
            *path=1;
        }

        else if (intercept == 2)
        {*path = 2;}
        else { *path =1;}

    }



    // this is where Friendly control can and findenemy functions requies.
    if (*path == 1){
        return_position_of_bogie.x = poi.x1;
        return_position_of_bogie.y = poi.y1;
    }
    else{
        return_position_of_bogie.x = poi.x2;
        return_position_of_bogie.y = poi.y2;
    }

    return_position_of_bogie.range_friendly=poi.range_to_friendly;


    position_of_bogie->x.insert(position_of_bogie->x.begin(),return_position_of_bogie.x);                            //updating values into the vector
    position_of_bogie->y.insert(position_of_bogie->y.begin(),return_position_of_bogie.y);
    return return_position_of_bogie;
}

IntPoints PurePursuit::interceptCalc(double radbogie,double rad_friendly,double xfriendly,double yfriendly){
    IntPoints int_points_;
    Velocities ve;
    const double MinVel= simulation_->V_TERM;
    double d = (-pow(rad_friendly,2)+pow(radbogie,2)+pow(xfriendly,2)+pow(yfriendly,2))/(2*xfriendly);
    double m = -4*(pow(yfriendly / xfriendly,2)+1)*(pow(d,2)-pow(radbogie,2));
    double root = pow((-2*d*(yfriendly / xfriendly)),2)+m;

    if(root>0){
        int_points_.y1=(-(-2*d*(yfriendly/xfriendly))+pow(root,0.5))/(2*(pow((yfriendly/xfriendly),2)+1));         //Using quadradic Formula to find the two interception points.
        int_points_.y2=(-(-2*d*(yfriendly/xfriendly))-pow(root,0.5))/(2*(pow((yfriendly/xfriendly),2)+1));
        int_points_.x1=d-int_points_.y1*(yfriendly/xfriendly);                                  //new interscept points when t=2s from t=1s
        int_points_.x2=d-int_points_.y2*(yfriendly/xfriendly);

        // If the intercept is outside the airspace (6000) then the friendly should go back to coordinate ( 0,0)
        if (int_points_.x1>(simulation_->AIRSPACE_SIZE)/2 || int_points_.y1>(simulation_->AIRSPACE_SIZE)/2||int_points_.x1<-(simulation_->AIRSPACE_SIZE)/2 || int_points_.y1<-(simulation_->AIRSPACE_SIZE)/2){
            ve.linearV_=MinVel;// making sure friendly does not go out of AIRSPACE_SIZE
            int_points_.x1=0;         // if bogie is outside airspace then friendly goes to (0,0)
            int_points_.y1=0;
        }
        if (int_points_.x2>(simulation_->AIRSPACE_SIZE)/2 || int_points_.y2>(simulation_->AIRSPACE_SIZE)/2||int_points_.x2<-(simulation_->AIRSPACE_SIZE)/2 || int_points_.y2<-(simulation_->AIRSPACE_SIZE)/2){
            ve.linearV_=MinVel;
            int_points_.x2=0;
            int_points_.y2=0;
        }
        return int_points_;
    }

}

RangeStamped PurePursuit::Extrapolate(std::vector<RangeStamped> x_bog_cord, double Timestamp_){   // When inialised, there is no values stored in the vector, hence the first range will be returned at t=0s
    RangeStamped return_value;
    return_value.timestamp = Timestamp_;

    if((x_bog_cord[0]).range == (x_bog_cord[1]).range){                                       // when nothing is moving and no distance updated.
        return_value.timestamp = (x_bog_cord[0]).range;                                           // take the first distance
        return return_value;
    }
    //Extrapolate equation
    return_value.range = (x_bog_cord[0]).range + ((Timestamp_ - (x_bog_cord[0]).timestamp)*((x_bog_cord[1]).range - (x_bog_cord[0]).range))/((x_bog_cord[1]).timestamp - (x_bog_cord[0]).timestamp);            // at t= infinity, it will constantly use extrapolation equation to find the Range for comparasiom.


    return return_value;                               // return the estimated point.
}


void PurePursuit::FriendlyControl(){
    BogiePos bogie_new_posi;
    Pose friendly_new_posi;
    Velocities v1;
    while(1){
        std::unique_lock<std::mutex> lock(mutex_bogie_position_);
        while (!bogie_position) {
            convar_bogie_position_.wait(lock);
        }
        mutex_bogie_position_.unlock();
        const double g = 9.8;                          //gravtational constance
        const double MaxG = simulation_->MAX_G ;        //Maximum G force 6G
        const double MaxV = simulation_->MAX_V;         // Max velocity 900m/s
        const double MinV = simulation_->V_TERM;        // Min Velocity 50m/s
        const  double MaxAngV = (MaxG*g)/MinV;             //Maximum Angular Velocity
        double offset;
        double angle_change;


        friendly_new_posi = getFriendlyPose();
        bogie_new_posi=getBogiePos();

        bogie_new_posi.x = bogie_new_posi.x - friendly_new_posi.position.x;                       //bogie position (distance) relative to friendly's
        bogie_new_posi.y = bogie_new_posi.y - friendly_new_posi.position.y;
        angle_change = friendly_new_posi.orientation-offset;                               //bogie's offset relative to friendly
        v1.linearV_ = (MaxG*g)/abs(v1.angularV_);                                                //calculate linear velocity when turing and moving

        // Friendly will find which quadrant the bogie is in

        if(bogie_new_posi.x>=0&&bogie_new_posi.y>=0){                                           // 1st quad
            offset = atan(bogie_new_posi.y/bogie_new_posi.x);
        }
        else if(bogie_new_posi.x<=0){                                                         // 2nd and 3rd quadrand assuming that any angle past 180 degrees is a negative degree
            offset = M_PI-asin(bogie_new_posi.y/pow(pow(bogie_new_posi.x,2)+pow(bogie_new_posi.y,2),0.5));
        }

        else{                                                                                // 4th quad (360 - (angle )
            offset = 2*M_PI-abs(asin(bogie_new_posi.y/pow(pow(bogie_new_posi.x,2)+pow(bogie_new_posi.y,2),0.5)));
        }

        // to identify which way to turn to get to the bogie fasest by figuring out what the angle difference between them and which quadrant they are in.
        if(angle_change >=0 && angle_change<=M_PI){   //cw
            angle_change = - abs(angle_change);
        }
        else if(angle_change >0 && angle_change> M_PI){    //ccw
            angle_change = M_PI*2-abs(angle_change);
        }
        else if(angle_change <=0 && angle_change>=-M_PI){  //cw
            angle_change = abs(angle_change);
        }
        else if(angle_change <0 && angle_change< -M_PI){   //ccw
            angle_change = -(2*M_PI-abs(angle_change));
        }

        //when the angle offset of 15 degrees (0.2618rad)
        if (abs(angle_change)>0.2618){
            v1.angularV_ = (abs(angle_change)*MaxAngV)/angle_change;
        }
        else{
            v1.angularV_= (MaxAngV*angle_change)/M_PI;
        }
        //ensuring that the friendly dont fly faster than 900m/s or slower than 50m/s
        if(v1.linearV_>MaxV){
            v1.linearV_ = MaxV;
        }
        if (bogie_new_posi.range_friendly < 90){  //Prevents overshooting when near the bogie.
            v1.linearV_ = MinV;
        }
        setVelocities(v1);
    }
}


void PurePursuit::findBogie(){                        //piecing everything together
    double path;
    IntPoints poi;
    BogieVal position_of_bogie;
    Pose friendly_coord;
    std::vector<RangeStamped> bogie_to_friendly_vec;
    RangeStamped base_range;
    RangeStamped friendly_range_from_bogie;
    while(1){
        std::unique_lock<std::mutex> lock(mutex_bogie_data_);                                 //mutex used to retrieve treads indvidually.
        while (!friendly_to_bogie_R_ || !base_to_bogie_R_) {
            range_recieved_.wait(lock);
        }

        bogie_to_friendly_vec = bogie_to_friendly_;
        base_range = bogie_range_from_base_;
        friendly_coord = friendly_pose_;
        friendly_to_bogie_R_=false;
        base_to_bogie_R_=false;
        lock.unlock();

        friendly_range_from_bogie = Extrapolate(bogie_to_friendly_vec,base_range.timestamp);
        poi = interceptCalc(base_range.range,friendly_range_from_bogie.range, friendly_coord.position.x, friendly_coord.position.y);  // get the intercept value with timestamp
        poi.range_to_friendly = bogie_to_friendly_vec.front().range;
        setBogiePos(FindPath(&path, poi, &position_of_bogie));
    }
}





//=========================================================//

void PurePursuit::DistanceFromBase(){     //Updates range when notified
    while(true){
        setBogieToBase(simulation_->rangeToBogieFromBase());
        setFriendlyPose(simulation_->getFriendlyPose());
        base_to_bogie_R_ = true;
        range_recieved_.notify_all();
    }
}

void PurePursuit::DistanceFromFriendly(){ // updateds range of Bogie to friendly Range when notified
    while(true){
        setBogieToFriendly(simulation_->rangeToBogieFromFriendly());
        setFriendlyPose(simulation_->getFriendlyPose());
        friendly_to_bogie_R_ = true;
        range_recieved_.notify_all();
    }
}

void PurePursuit::Watchdog(){
    Velocities cur_req_vel;
    while(true){
        cur_req_vel = getVelocities();
        if(!simulation_->controlFriendly(cur_req_vel.linearV_, cur_req_vel.angularV_)){

        }
    }
}
