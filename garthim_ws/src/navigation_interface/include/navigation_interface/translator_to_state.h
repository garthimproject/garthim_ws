#ifndef __TRANSLATOR_TO_STATE_NAVIGATION_INTERFACE_H__
#define __TRANSLATOR_TO_STATE_NAVIGATION_INTERFACE_H__

#include <ros/ros.h>
#include <rl_msgs/RLVariable.h>
#include <robot_interface/task_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <exception>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rl_msgs/GetRandom.h>
#include <navigation_interface/target_generator.h>
#include <navigation_interface/target_generator_scaffolder.h>
#include <navigation_interface/translator_action_to_twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <rl_msgs/UnwrapState.h>

namespace navigation_interface {
    /**
     * @brief Struct that enables using bumper events before being received.
     * 
     */
    struct CollisionStatusStruct {
        kobuki_msgs::BumperEvent event;
        bool is_received;
    };
    typedef struct CollisionStatusStruct CollisionStatus;
    /**
     * @brief Class with the task of translating real-life measures to rl_msgs::State
     * 
     */
    class TranslatorToState {
        private:
            /**
             * @brief (Debugging purposes) Publisher of a point cloud containing the 
             * nearest points to the robot in each of the laser sections.
             * 
             */
            ros::Publisher pub_point_laser_; 
            /**
             * @brief (Debugging purposes) Publisher of a point cloud containing the 
             * all the poibnts of the laser after a manually done translation of frames.
             * 
             */
            ros::Publisher pub_scan_;
            /**
             * @brief (Debugging purposes) Messages for a point cloud of the selected 
             * points to build the current agent state and a point cloud encapsulating
             * all laser.
             * 
             */
            sensor_msgs::PointCloud pc_, pc_scan_;
            /**
             * @brief Distance at which the last discretization level starts.
             * Or in continuous case, at all.
             * 
             */
            double radius_start_last_level_;
            /**
             * @brief Radius of tolerance to reach a goal. If distance to target is less
             * target is considered to be reached.
             * 
             */
            double radius_goal_states_;
            /**
             * @brief Generic measure of distance between states after goal level.
             * 
             */
            double radius_unit_states_;
            /**
             * @brief Measure for less important radius levels, it begins after 3rd level.
             * 
             */
            double radius_long_distance_states_;
            /**
             * @brief Total number of radius levels in which to divide target distance in order to form the state of the agent.
             * 
             */
            double radius_levels_;
            /**
             * @brief Vector containing the computed distance limit of each radius level.
             * 
             */
            std::vector<double> radiuses_;
            /**
             * @brief Number of divisions used to discretize agent orientation.
             * 
             */
            double orientation_levels_;
            double spencer_total_persons_;
            double spencer_orientation_levels_;
            double spencer_orientation_vel_levels_;
            double spencer_orientation_vel_unit_;
            double spencer_depth_levels_;
            double spencer_depth_unit_;
            double spencer_depth_collision_;
            /**
             * @brief Number of orientation sections to divide laser measures in order to discretize them.
             * 
             */
            double scanner_orientation_levels_;
            /**
             * @brief Number of depth levels to divide and discretize laser measures.
             * 
             */
            double scanner_depth_levels_;
            /**
             * @brief Distance unit in which to divide range measures.
             * 
             */
            double scanner_depth_unit_;
            /**
             * @brief Distance to obstacles in which to consider collisions.
             * 
             */
            double radius_collision_;
            /**
             * @brief Total of choosable durations for an action.
             * 
             */
            double time_action_levels_;
            /**
             * @brief Flag indicating if discretization is performed.
             * 
             */
            bool is_discrete_;
            /**
             * @brief Total states considered to discretize agent pose in relation to a target position.
             * 
             */
            uint64_t spatial_states_;
            /**
             * @brief Pose for navigation target. Orientation is ignored.
             * 
             */
            geometry_msgs::PoseStamped target_;
            /**
             * @brief Current agent pose.
             * 
             */
            geometry_msgs::PoseStamped robot_pose_;
            /**
             * @brief Object in charge of generating new targets after episodes are completed.
             * 
             */
            TargetGeneratorScaffolder tg_;
            /**
             * @brief Buffer used to listen to transform tree.
             * 
             */
            tf2_ros::Buffer buf_; 
            /**
             * @brief At the beginning of the experiment this flag enables to listen to 
             * tf2: "hokuyo_link" -> "base_footprint".
             * This tf is only obtained once, since laser frame is fixed to base_link frame.
             */
            bool is_tf_laser_initialized_;
            /**
             * @brief Flag indicating if translator is receiving laser info to compute states.
             * 
             */
            bool is_used_laser_;
            bool is_used_spencer_;
            /**
             * @brief Flag indicating whether translator uses bumper messages to compute states.
             * 
             */
            bool is_using_bumpers_;
            /**
             * @brief Flag indicating if debug messages are shown.
             * 
             */
            bool debug_;
            /**
             * @brief This tf is used to translate poses from 
             * tf msg: "hokuyo_link" -> "base_footprint".
             * This tf is only obtained once, since laser frame is fixed to base_link frame.
             * 
             */
            geometry_msgs::TransformStamped tf_laser_;
            /**
             * @brief Service server to obtain the value of each variable composing a state.
             * 
             */
            ros::ServiceServer server_unwrap_state_;
            /**
             * @brief Private method to be delegated the builder functionality.
             * 
             */
            void initialize();
        public:
            inline TranslatorToState() : buf_(ros::Duration(20)) {
                initialize();
            }
            inline TranslatorToState(unsigned seed) : buf_(ros::Duration(20)) {
                initialize();
            }
            ~TranslatorToState();
            /**
             * @brief Obtain current state based on odometry taking into account the time it takes to perform an action.
             * This time is selected using index_time_action on a previously known array.
             * 
             * @param odometry Assumption on the pose of the agent.
             * @param a_t The translator to twist used in this package. It is needed to obtain duration time info.
             * @param state out-parameter. State of the agent.
             */
            void getStateDiscrete(const geometry_msgs::Pose& odometry, const TranslatorActionToTwist& a_t, rl_msgs::State& state);
            /**
             * @brief Obtain current state based on odometry, scanner and bumper events taking into account the time it takes 
             * to perform an action. This time is selected using index_time_action on a previously known array.
             * 
             * @param odometry Assumption on the pose of the agent.
             * @param scan Scanner measure at the moment.
             * @param bumper Bumper event info.
             * @param a_t The translator to twist used in this package. It is needed to obtain duration time info.
             * @param state out-parameter. State of the agent.
             */
            void getStateDiscrete(const geometry_msgs::Pose& odometry, const sensor_msgs::LaserScan& scan, const CollisionStatus& bumper, const TranslatorActionToTwist& a_t, rl_msgs::State& state);
            /**
             * @brief Obtain current state based on odometry, scanner and bumper events taking into account the time it takes 
             * to perform an action. This time is selected using index_time_action on a previously known array.
             * 
             * @param odometry Assumption on the pose of the agent from gazebo.
             * @param odometry_from_topic Assumption on the pose of the agent from topic.
             * @param pers_msg Info of the persons detected by Spencer blob-detector.
             * @param scan Scanner measure at the moment.
             * @param bumper Bumper event info.
             * @param a_t The translator to twist used in this package. It is needed to obtain duration time info.
             * @param state out-parameter. State of the agent.
             */
            void getStateDiscrete(const geometry_msgs::Pose& odometry,const geometry_msgs::Pose& odometry_from_topic,
             const spencer_tracking_msgs::TrackedPersons& pers_msg, const sensor_msgs::LaserScan& scan,
             const CollisionStatus& bumper, const TranslatorActionToTwist& a_t, rl_msgs::State& state);
            /**
             * @brief (Not implemented yet) Obtain current continuous state based on odometry.
             * 
             * @param odometry Assumption on the pose of the agent.
             * @param state out-parameter. State of the agent.
             */
            void getStateContinuous(const geometry_msgs::Pose& odometry, rl_msgs::State& state);
            /**
             * @brief Function to discretize robot orientation.
             * 
             * @param inc_x x component of distance to target.
             * @param inc_y y component of distance to target.
             * @param odometry Assumption on the pose of the agent.
             * @return int64_t State containing the discretization of agent orientation.
             */
            inline static int64_t getOrientationId(double inc_x, double inc_y, double levels, const geometry_msgs::Pose& odometry) {
                double theta = atan2(inc_y,inc_x);
                theta = theta - 2*M_PI*floor(theta/(2*M_PI));
                double orientation = theta - tf::getYaw(odometry.orientation);
                orientation = orientation - 2*M_PI*floor(orientation/(2*M_PI));
                double orientation_unit = 2*M_PI/levels;
                return trunc(orientation/orientation_unit);
            }
            /**
             * @brief Method to reset the navigation goal.
             * 
             * @param status The status of the current episode.
             * @return true if scaffolding is ready to be performed.
             * @return false otherwise.
             */
            inline bool reset(const uint8_t status=0) {
                return tg_.getNewTarget(robot_pose_.pose,target_.pose, status);
            }
            /**
             * @brief Method to set cylinder obstacles in target generator.
             * 
             * @param pose Pose of the centroid of the cylinder.
             */
            inline void setCylinder(const geometry_msgs::Pose& pose) {
                tg_.setCylinder(pose);
            }
            /**
             * @brief Method to set ped. obstacles in target generator.
             * 
             * @param pose Pose of the centroid of the pedestrian.
             * @param id id of pedestrian.
             */
            inline void setPedestrian(const geometry_msgs::Pose& pose, int id) {
                tg_.setPedestrian(pose, id);
            }
            /**
             * @brief Set the walls limiting the room where the agent navigates.
             * 
             * @param pose Pose of the center of the room.
             */
            inline void setWalls(const geometry_msgs::Pose& pose) {
                tg_.setWalls(pose);
            }
            /**
             * @brief Generic method that stores current odometry pose and delegates 
             * the translation of said pose to state. 
             * 
             * @param odometry Odometry of the agent as a pose msg.
             * @param a_t The translator to twist used in this package. It is needed to obtain duration time info.
             * @param state out-parameter. State obtained from odometry.
             */
            inline void translate(const geometry_msgs::Pose& odometry, const TranslatorActionToTwist& a_t, rl_msgs::State& state) {
                copyPose(odometry, robot_pose_.pose);
                is_discrete_ ? getStateDiscrete(odometry, a_t, state)
                    : getStateContinuous(odometry, state);
            }
            /**
             * @brief Generic method that stores current pose and delegates the translation
             * of said pose to state using scanner and bumpers on top of that.
             * 
             * @param odometry Odometry of the agent as a pose msg.
             * @param scan Scanner measure at the moment.
             * @param bumper Bumper event info.
             * @param a_t The translator to twist used in this package. It is needed to obtain duration time info.
             * @param state out-parameter. State obtained from odometry.
             */
            inline void translate(const geometry_msgs::Pose& odometry, const sensor_msgs::LaserScan& scan, const CollisionStatus& bumper, const TranslatorActionToTwist& a_t, rl_msgs::State& state) {
                copyPose(odometry, robot_pose_.pose);
                is_discrete_ ? getStateDiscrete(odometry,scan,bumper, a_t, state)
                    : getStateContinuous(odometry, state);
                //TODO
            }
            /**
             * @brief Generic method that stores current pose and delegates the translation
             * of said pose to state using scanner and bumpers on top of that.
             * 
             * @param odometry Assumption on the pose of the agent from gazebo.
             * @param odometry_from_topic Assumption on the pose of the agent from topic.
             * @param pers_msg Info of the persons detected by Spencer blob-detector.
             * @param scan Scanner measure at the moment.
             * @param bumper Bumper event info.
             * @param a_t The translator to twist used in this package. It is needed to obtain duration time info.
             * @param state out-parameter. State of the agent.
             */
            inline void translate(const geometry_msgs::Pose& odometry, const geometry_msgs::Pose& odometry_from_topic, const spencer_tracking_msgs::TrackedPersons& spen_msg, const sensor_msgs::LaserScan& scan, const CollisionStatus& bumper, const TranslatorActionToTwist& a_t, rl_msgs::State& state) {
                copyPose(odometry, robot_pose_.pose);
                is_discrete_ ? getStateDiscrete(odometry,odometry_from_topic,spen_msg,scan,bumper, a_t, state)
                    : getStateContinuous(odometry, state);
                //TODO
            }
            /**
             * @brief Set robot_pose_.pose to odometry.
             * 
             * @param odometry Odometry of the robot.
             */
            inline void setRobotPose(const geometry_msgs::Pose& odometry) {
                copyPose(odometry, robot_pose_.pose);
            }
            /**
             * @brief Get target_pose copying each component to the out-parameter.
             * 
             * @param target out-parameter
             */
            inline void getTarget(geometry_msgs::Pose& target) {
                target.position.x = target_.pose.position.x;
                target.position.y = target_.pose.position.y;
                target.position.z = target_.pose.position.z;
                target.orientation.x = target_.pose.orientation.x;
                target.orientation.y = target_.pose.orientation.y;
                target.orientation.z = target_.pose.orientation.z;
                target.orientation.w = target_.pose.orientation.w;
            }
            /**
             * @brief Set target_pose copying each component from the parameter.
             * 
             * @param target Pose of the goal target
             */
            inline void setTarget(const geometry_msgs::Pose& target) {
                target_.pose.position.x = target.position.x;
                target_.pose.position.y = target.position.y;
                target_.pose.position.z = target.position.z;
                target_.pose.orientation.x = target.orientation.x;
                target_.pose.orientation.y = target.orientation.y;
                target_.pose.orientation.z = target.orientation.z;
                target_.pose.orientation.w = target.orientation.w;
            }
            /**
             * @brief Function to obtain if target has being reached. State is considered final.
             * 
             * @param state Current state
             * @return true if target has been reached
             * @return false otherwise
             */
            inline bool isFinal(const rl_msgs::RLVariable& state) {
                return is_discrete_&& state.as_integer[0] % spatial_states_ < orientation_levels_;
            }
            /**
             * @brief Check if a point, its components being x,y, is inside a radius of collision.
             * 
             * @param x x component of a point.
             * @param y y component of a point.
             * @param radius_collision Maximum distance in which exists a collision with an obstacle or
             * is dangerously close.
             * @return true if a point is dangerously close to an obstacle or if it is colliding to it.
             * @return false otherwise.
             */
            inline static bool checkLaserCollision(const double x, const double y, const double radius_collision) {
                ROS_INFO("x:%f, y;%f, b1:%f, r_2:%f", x,y,pow(x,2) + pow(y,2), pow(0.30,2));
                return pow(x,2) + pow(y,2) < pow(radius_collision,2);
            }
            inline static void copyPose(const geometry_msgs::Pose& in, geometry_msgs::Pose& out) {
                out.position.x = in.position.x;
                out.position.y = in.position.y;
                out.position.z = in.position.z;
                out.orientation.x = in.orientation.x;
                out.orientation.y = in.orientation.y;
                out.orientation.z = in.orientation.z;
                out.orientation.w = in.orientation.w;
            }
            /**
             * @brief Get a discretization of each scanner section based on its depth.
             * 
             * @param scan Scanner message.
             * @param out out-parameter containing each discretization.
             */
            void getStatesScanner(const sensor_msgs::LaserScan& scan, std::vector<int64_t>& out);

            /**
             * @brief Get a discretization of the distance to persons detected by spencer received message.
             * 
             * @param persons Persons detected by spencer.
             * @param out out-parameter containing each discretization.
             */
            void getStatesSpencer(const geometry_msgs::Pose& odometry, const spencer_tracking_msgs::TrackedPersons& persons, std::vector<int64_t>& out);

            /**
             * @brief Get the state por the bumper sensors.
             * 
             * @param bumper Bumper wrapper of bumper event.
             * @return int64_t State
             */
            inline static int64_t getStatesBumper(const CollisionStatus& bumper) {
                int64_t state = 0;
                if (bumper.is_received) {
                    uint8_t pressed = bumper.event.state;
                    state = pressed + pressed*bumper.event.bumper;
                }
                return state;
            }
            /**
             * @brief Function to discretize depth to an obstacle.
             * 
             * @param r Distance to obstacle.
             * @return double discretization of r.
             */
            inline double getDigitStateForScannerRay(double r) {
                double state;
                if (r < radius_collision_) {
                    state = 0;
                } else {
                    r -= radius_collision_;
                    double max_divs = this->scanner_depth_levels_;
                    double d = floor(r/scanner_depth_unit_) + 1;
                    if (d >= max_divs) {
                        state = max_divs - 1;
                    } else {
                        state = d;
                    }
                }
                return state;
            }
            /**
             * @brief Transform laser measure from laser frame and polar coordinates.
             * 
             * @param header Header of scan message, used to obtain frame_id of laser and stamp.
             * @param laser_measure Depth measure from laser.
             * @param angle Orientation measure from laser.
             * @param x out-parameter x component.
             * @param y out-parameter y component.
             */
            void transformLaserToCartesian(const std_msgs::Header& header, const float laser_measure, const float angle, double &x, double &y);
            /**
             * @brief Obtain a representation of the state of the robot in relation to scanner info.
             * 
             * @param scan Laser scanner message.
             * @param bumper Bumper status
             * @param collis out-parameter indicating if a collision has been detected.
             * @return int64_t number representation of a state.
             */
            int64_t getObstaclesState(const sensor_msgs::LaserScan& scan, const CollisionStatus& bumper, bool& collis);
            /**
             * @brief Service callback that obtains the value of each variable of a state.
             * 
             * @param req An rl_msgs::State containing a state desired to be unwrapped.
             * @param res A vector containing the value of each variable for request.
             * @return true if somehow service call has failed.
             * @return false otherwise.
             */
            bool unwrapState(rl_msgs::UnwrapStateRequest& req, 
            rl_msgs::UnwrapStateResponse& res);
    };
}
#endif
