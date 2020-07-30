#ifndef CNR_CONTROLLER_INTERFACE__CNR_KINEMATIC_STATUS_H
#define CNR_CONTROLLER_INTERFACE__CNR_KINEMATIC_STATUS_H

#include <eigen3/Eigen/Core>

namespace cnr_controller_interface
{

struct KinematicStatus
{
  Eigen::VectorXd  q;
  Eigen::VectorXd  qd;
  Eigen::VectorXd  qdd;
  Eigen::VectorXd  effort;
  void resize(const size_t& nAx)
  {
    q     .resize(nAx);
    qd    .resize(nAx);
    qdd   .resize(nAx);
    effort.resize(nAx);
    setZero();
  }
  void setZero()
  {
    q     .setZero();
    qd    .setZero();
    qdd   .setZero();
    effort.setZero();
  }
  KinematicStatus() = default;
  KinematicStatus(const KinematicStatus& cpy)
  {
    q     = q     ;
    qd    = qd    ;
    qdd   = qdd   ;
    effort= effort;
  }
  KinematicStatus& operator=(const KinematicStatus& rhs)
  {
    this->q     = rhs.q     ;
    this->qd    = rhs.qd    ;
    this->qdd   = rhs.qdd   ;
    this->effort= rhs.effort;
    return *this;
  }
};

template<class T>
bool extract( const T* hw, KinematicStatus& st)
{
  return false;
}


template<class T>
bool extract( const KinematicStatus& cmd, T* hw)
{
  return false;
}


}
#endif  // CNR_CONTROLLER_INTERFACE__CNR_HANDLES_UTILS_H



