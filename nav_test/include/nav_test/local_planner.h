 /** include the libraries you need in your planner here */
 /** for global path planner interface */
 #include <ros/ros.h>
 #include <nav_core/base_local_planner.h>

 #ifndef LOCAL_PLANNER_CPP
 #define LOCAL_PLANNER_CPP

 namespace local_planner {

 class LocalPlanner : public nav_core::BaseLocalPlanner {
 public:

  LocalPlanner();
  LocalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /** overridden classes from interface nav_core::BaseLocalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan
               );
  };
 };
 #endif

