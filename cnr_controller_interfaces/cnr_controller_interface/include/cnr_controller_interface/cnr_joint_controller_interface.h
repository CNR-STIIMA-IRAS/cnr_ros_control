#ifndef __CNR__JOINT_CONTROLLER_INTERFACE__
#define __CNR__JOINT_CONTROLLER_INTERFACE__

#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_controller_interface/cnr_controller_interface.h>


namespace cnr_controller_interface
{

  /**
   *
   *
   *
   * Base class to log the controller status
   */
  template< class T >
  class JointController: public cnr_controller_interface::Controller< T >
  {
  public:

    ~JointController( )
    {
      CNR_TRACE_START(*cnr_controller_interface::Controller< T >::m_logger);
    }

    virtual bool doInit       ( )                                                            { return true; }
    virtual bool doStarting   ( const ros::Time& /*time*/ )                                  { return true; }
    virtual bool doUpdate     ( const ros::Time& /*time*/, const ros::Duration& /*period*/ ) { return true; }
    virtual bool doStopping   ( const ros::Time& /*time*/ )                                  { return true; }
    virtual bool doWaiting    ( const ros::Time& /*time*/ )                                  { return true; }
    virtual bool doAborting   ( const ros::Time& /*time*/ )                                  { return true; }

    const std::vector< std::string >&        getJointNames( ) const { return m_joint_names; }
    std::shared_ptr<cnr_logger::TraceLogger> logger() {return cnr_controller_interface::Controller< T >::m_logger; }
  public:

    bool enterInit (  )
    {
      CNR_TRACE_START(*Controller<T>::m_logger);
      if( !Controller<T>::enterInit( ) )
      {
        CNR_RETURN_FALSE(*Controller<T>::m_logger);
      }

      XmlRpc::XmlRpcValue value;
      if (!Controller<T>::getControllerNh().getParam("controlled_joint", value) && !Controller<T>::getControllerNh().getParam("controlled_joints", value) )
      {
        CNR_RETURN_FALSE(*Controller<T>::m_logger, "The param " + Controller<T>::getControllerNamespace() + "/controlled_joint(s) not defined");
      }

      if( value.getType() == XmlRpc::XmlRpcValue::TypeArray )
      {
        if( value[0].getType( ) == XmlRpc::XmlRpcValue::TypeString )
        {
          for(int i=0; i<value.size(); i++)
          {
            m_joint_names.push_back((std::string)(value[i]) );
          }
        }
        else
        {
          CNR_RETURN_FALSE(*Controller<T>::m_logger, "The param " + Controller<T>::getControllerNamespace() + "/controlled_joint(s) bad formed");
        }
      }
      else if( value.getType( ) == XmlRpc::XmlRpcValue::TypeString )
      {
        m_joint_names.push_back((std::string)(value) );
      }
      else
      {
        CNR_RETURN_FALSE(*Controller<T>::m_logger, "The param " + Controller<T>::getControllerNamespace() + "/controlled_joint(s) bad formed");
      }

      m_nAx = m_joint_names.size();
      for (std::string name: m_joint_names)
      {
        try
        {
          Controller<T>::m_hw->getHandle(name);
        }
        catch(...)
        {
          CNR_RETURN_FALSE(*Controller<T>::m_logger, "Controller '" + Controller<T>::getControllerNamespace() + "' failed in init. The controlled joint named '" + name+"' is not managed by hardware_interface");
        }
      }

      CNR_RETURN_BOOL(*Controller<T>::m_logger, Controller<T>::dump_state( ) );
    }

    const std::string& getJointName() const { return m_joint_names; }
 protected:

    std::vector<std::string>        m_joint_names;
    size_t                          m_nAx;

  };

} // cnr_controller_interface
#endif
