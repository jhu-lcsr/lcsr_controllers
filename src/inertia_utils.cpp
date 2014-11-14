
#include <lcsr_controllers/inertia_utils.h>

namespace lcsr_controllers {

  KDL::RigidBodyInertia inertiaMsgToKDL(const telemanip_msgs::Inertia &i)
  {
    return KDL::RigidBodyInertia(
        i.m,
        KDL::Vector(i.com.x, i.com.y, i.com.z),
        KDL::RotationalInertia(
            i.ixx, i.iyy, i.izz,
            i.ixy, i.ixz, i.iyz));
  }

  bool check_inertia(const KDL::RigidBodyInertia &inertia) 
  {
    return inertia.getMass() > 1.0E-4;
  }

  void interpolate_inertia(
      KDL::RigidBodyInertia &interpolated,
      const KDL::RigidBodyInertia &current,
      const KDL::RigidBodyInertia &target,
      const double mass_rate,
      const double cog_rate,
      const ros::Duration dt)
  {
    // Compute mass update
    const double dm_total = target.getMass() - current.getMass();
    const double dm_abs = fabs(dm_total);
    const double dm = std::min(dm_abs, mass_rate*dt.toSec()) * (dm_total > 0 ? 1.0 : -1.0);

    // Compute COG update
    KDL::Vector dcog_total = target.getCOG() - current.getCOG();
    const double dcog_norm = dcog_total.Normalize();
    const KDL::Vector dcog = std::min(dcog_norm, cog_rate*dt.toSec()) * dcog_total;

    // Set the interpolated inertia
    interpolated = KDL::RigidBodyInertia(
        current.getMass() + dm,
        current.getCOG() + dcog,
        target.getRotationalInertia());
  }


  //! Update inertia map
  bool AttachedInertiaMap::update(const telemanip_msgs::AttachedInertia &attached_inertia)
  {
    switch(attached_inertia.action) {
      case telemanip_msgs::AttachedInertia::UPDATE:
        return set_attached_inertia(
            inertiaMsgToKDL(attached_inertia.inertia),
            attached_inertia.ns,
            attached_inertia.id);
      case telemanip_msgs::AttachedInertia::DELETE:
        return del_attached_inertia(
            attached_inertia.ns,
            attached_inertia.id);
        break;
    };

    return false;
  }

  //! Sum all of the inertias
  KDL::RigidBodyInertia AttachedInertiaMap::sum() const
  {
    KDL::RigidBodyInertia total_inertia;

    // iterate over ns
    for(ns_map_t_::const_iterator ns_it = inertias_.begin();
        ns_it != inertias_.end();
        ++ns_it)
    {
      // iterate over id
      for(id_map_t_::const_iterator id_it = ns_it->second.begin();
          id_it != ns_it->second.end();
          ++id_it)
      {
        // add to total
        total_inertia = total_inertia + id_it->second;
      }
    }

    return total_inertia;
  }

  bool AttachedInertiaMap::set_attached_inertia(
      const KDL::RigidBodyInertia &inertia,
      const std::string &ns,
      const size_t id)
  {
    // Check if inertia is well-posed
    if(!check_inertia(inertia)) {
      return false;
    }

    // Get id map from ns map
    ns_map_t_::iterator ns_it = inertias_.find(ns);
    if(ns_it == inertias_.end()) {
      ns_it = inertias_.insert(ns_map_t_::value_type(ns, id_map_t_())).first;
    }

    // Get inertia from id map
    id_map_t_ &id_map = ns_it->second;
    id_map_t_::iterator id_it = id_map.find(id);
    if(id_it == id_map.end()) {
      // Create a new inertia element
      id_it = id_map.insert(id_map_t_::value_type(id, inertia)).first;
    } else {
      // Update the existing inertia element
      id_it->second = inertia;
    }

    return true;
  }

  bool AttachedInertiaMap::get_attached_inertia(
      KDL::RigidBodyInertia &inertia,
      const std::string &ns,
      const size_t id) const
  {
    // Get id map from ns map
    ns_map_t_::const_iterator ns_it = inertias_.find(ns);
    if(ns_it == inertias_.end()) {
      return false;
    }

    // Get inertia from id map
    const id_map_t_ &id_map = ns_it->second;
    id_map_t_::const_iterator id_it = id_map.find(id);
    if(id_it == id_map.end()) {
      return false;
    }

    // Return the inertia by reference
    inertia = id_it->second;

    return true;
  }

  bool AttachedInertiaMap::del_attached_inertia(
      const std::string &ns,
      const size_t id)
  {
    // Get id map from ns map
    ns_map_t_::iterator ns_it = inertias_.find(ns);
    if(ns_it == inertias_.end()) {
      return false;
    }

    // Get inertia from id map
    id_map_t_ &id_map = ns_it->second;
    id_map_t_::iterator id_it = id_map.find(id);
    if(id_it == id_map.end()) {
      return false;
    }

    // Remove id
    id_map.erase(id_it);

    // Remove ns if necessary
    if(ns_it->second.empty()) {
      inertias_.erase(ns_it);
    }

    return true;
  }

}
