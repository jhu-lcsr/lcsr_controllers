#ifndef __LCSR_CONTROLLERS_INERTIA_UTILS_H
#define __LCSR_CONTROLLERS_INERTIA_UTILS_H

#include <string>
#include <map>
#include <ros/ros.h>

#include <telemanip_msgs/AttachedInertia.h>

#include <kdl/rigidbodyinertia.hpp>

namespace lcsr_controllers {

  //! Convert a ROS inertia message to a KDL RigidBodyInertia
  KDL::RigidBodyInertia inertiaMsgToKDL(const telemanip_msgs::Inertia &i);

  //! Check if an inertia message is well-posed
  bool check_inertia(const KDL::RigidBodyInertia &inertia);

  //! Interpolate between two rigid body inertias at a given rate
  void interpolate_inertia(
      KDL::RigidBodyInertia &interpolated,
      const KDL::RigidBodyInertia &current,
      const KDL::RigidBodyInertia &target,
      const double mass_rate,
      const double cog_rate,
      const ros::Duration dt);

  class AttachedInertiaMap
  {
  public:

    //! Update inertia map from a ROS attached inertia message
    bool update(const telemanip_msgs::AttachedInertia &attached_inertia);

    //! Get a store attached inertia
    bool set_attached_inertia(
        const KDL::RigidBodyInertia &inertia,
        const std::string &ns,
        const size_t id);

    //! Get a store attached inertia
    bool get_attached_inertia(
        KDL::RigidBodyInertia &inertia,
        const std::string &ns,
        const size_t id) const;

    //! Get a store attached inertia
    bool del_attached_inertia(
        const std::string &ns,
        const size_t id);

    //! Sum all of the inertias
    bool sum(KDL::RigidBodyInertia &total_inertia, double max=0.0) const;

  private:

    typedef std::map<size_t, KDL::RigidBodyInertia> id_map_t_;
    typedef std::map<std::string, id_map_t_> ns_map_t_;
    ns_map_t_ inertias_;
  };
}

#endif // ifndef __LCSR_CONTROLLERS_INERTIA_UTILS_H
