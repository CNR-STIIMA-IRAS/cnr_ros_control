# Hardware Nodelet Interface

The package provides two classes to ease the configuration and the life-cycle management of the `nodelet`.

## RobotHwDriverInterface Class

The class `cnr_hardware_driver_interface::RobotHwDriverInterface : public nodelet::Nodelet`  packs a `RobotHW` inside a `nodelet`. Basically, the Class implements the `onInit` function inherited from the `public nodelet::Nodelet`

```cpp
void RobotHwDriverInterface::onInit()
{
  m_hw_namespace = getPrivateNodeHandle().getNamespace();
  m_hw_name      = extractRobotName(m_hw_namespace);
  m_updater.setHardwareID(m_hw_name);
  m_logger.reset(new cnr_logger::TraceLogger("NL_" + m_hw_name, m_hw_namespace));

  try
  {
    CNR_TRACE_START(m_logger);
    if (enterOnInit() && doOnInit() && exitOnInit())
    {
      m_stop_update_thread = m_stop_diagnostic_thread = false;
      CNR_RETURN_OK(m_logger, void());
    }
  }
  catch (std::exception& e)
  {
    CNR_RETURN_NOTOK(m_logger, void(), m_hw_name + ": ExitOnInit failed. Exception caught: " + std::string(e.what()));
  }
  catch (...)
  {
    CNR_RETURN_NOTOK(m_logger, void(), m_hw_name + ": ExitOnInit failed. UNhandled Exception");
  }
}
```

The implementation is split in three sub-methods `enterOnInit()`, `doOnInit()` and `exitOnInit()`, so the user can derive and modify `doOnInit()` preserving all the functionalities provided by the class.

The `RobotHwDriverInterface::enterOnInit()` prepares the classes, the diagnosis module, check the sampling period parameters, and create the `plugin` loader of the `RobotHw` class

```cpp
bool RobotHwDriverInterface::enterOnInit()
{
  CNR_TRACE_START(m_logger);
  m_stop_update_thread = m_stop_diagnostic_thread = false;

  // getNodeHandle(), parent nodelet handle (typically "/")  --- getPrivateNodeHandle() -> /nodelet_name/
  m_nh     = getNodeHandle();
  m_hw_nh  = getPrivateNodeHandle();

  double sampling_period = 0.001;
  if (!rosparam_utilities::get(m_hw_nh.getNamespace() +"/sampling_period", sampling_period))
  {
    CNR_WARN(m_logger, m_hw_namespace + "/sampling_period' does not exist, set equal to 0.001");
    sampling_period = 1.0e-3;
  }

  m_period              = ros::Duration(sampling_period);

  m_time_span_tracker.emplace("cycle", new realtime_utilities::TimeSpanTracker(int(10.0 / sampling_period), sampling_period));
  m_time_span_tracker.emplace("read", new realtime_utilities::TimeSpanTracker(int(10.0 / sampling_period), sampling_period));
  m_time_span_tracker.emplace("write", new realtime_utilities::TimeSpanTracker(int(10.0 / sampling_period), sampling_period));
  m_time_span_tracker.emplace("update", new realtime_utilities::TimeSpanTracker(int(10.0 / sampling_period), sampling_period));

  m_robot_hw_plugin_loader.reset(
      new pluginlib::ClassLoader<cnr_hardware_interface::RobotHW>("cnr_hardware_interface",
                                                                  "cnr_hardware_interface::RobotHW"));

  CNR_RETURN_TRUE(m_logger);
}
```

The `RobotHwDriverInterface::doOnInit()` method creates the `RobtHw` objects exploiting the `plugin` loader. The function can be inherited for specific implementation. The `RobotHw` needs an empty Constructor.

```cpp
bool RobotHwDriverInterface::doOnInit()
{
  CNR_TRACE_START(m_logger);
  std::string robot_type;
  try
  {
    if (!rosparam_utilities::get(m_hw_nh.getNamespace() +"/type", robot_type))
    {
      CNR_FATAL(m_logger, "The param '" << m_hw_nh.getNamespace() << "/type' is missing! Abort.");
      CNR_RETURN_FALSE(m_logger);
    }
    m_robot_hw_plugin_loader->loadLibraryForClass(robot_type);
    m_hw = m_robot_hw_plugin_loader->createInstance(robot_type);
  }
  catch (std::exception& e)
  {
    CNR_RETURN_FALSE(m_logger, "OnInit of FakeRobotHw failed! what: " + std::string(e.what()));
  }
  CNR_RETURN_TRUE(m_logger);
}
```

Finally, `RobotHwDriverInterface::exitOnInit()` creates two running threads: a diagnostic thread, and the control thread. Furthermore, the `ControllerManager` [(ros_control)](http://wiki.ros.org/ros_control) is created, in order to allow the dynamic loading/unloading of the controller. 

```cpp
bool RobotHwDriverInterface::exitOnInit()
{
  CNR_TRACE_START(m_logger);
  CNR_DEBUG(m_logger, "Triggering the start of the main thread");

  if (m_hw == nullptr)
  {
    CNR_RETURN_FALSE(m_logger, "The RobotHw has not been properly initialized in the doOnInit() function. Abort.");
  }

  if (!m_hw->init(m_nh, m_hw_nh))
  {
    CNR_RETURN_FALSE(m_logger, "The RobotHw '" + m_hw_name + "'Initialization failed. Abort.");
  }

  m_cm.reset(new controller_manager::ControllerManager(m_hw.get(), m_hw_nh));

  m_update_thread_state = ON_INIT;
  ros::Time start       = ros::Time::now();
  m_update_thread       = std::thread(&cnr_hardware_driver_interface::RobotHwDriverInterface::controlUpdateThread, this);
  while (m_update_thread_state != RUNNING)
  {
    if (m_update_thread_state == ON_ERROR)
    {
      CNR_RETURN_FALSE(m_logger, "The update thread is in Error. Abort.");
    }
    if (m_update_thread_state == EXPIRED)
    {
      CNR_RETURN_FALSE(m_logger, "The update thread is expired. Something went wrong during the start. Abort.");
    }
    if ((ros::Time::now() - start).toSec() > 10.0)
    {
      CNR_RETURN_FALSE(m_logger, "Timeout Expired. The update thread did not yet started. Abort.");
    }
    ros::Duration(0.05).sleep();
  }

  m_diagnostics_thread_state = ON_INIT;
  start                      = ros::Time::now();
  m_diagnostics_thread       = std::thread(&cnr_hardware_driver_interface::RobotHwDriverInterface::diagnosticsThread, this);
  while (m_diagnostics_thread_state != RUNNING)
  {
    if (m_diagnostics_thread_state == ON_ERROR)
    {
      CNR_RETURN_FALSE(m_logger, "Main thread in ERROR");
    }
    if ((ros::Time::now() - start).toSec() > 10.0)
    {
      CNR_RETURN_FALSE(m_logger, "Timeout Exipred. Main Thread did not started yet. Abort.");
    }
    ros::Duration(0.05).sleep();
  }
  CNR_DEBUG(m_logger, "The main is running");
  CNR_RETURN_TRUE(m_logger);
}
```

## NodeletManagerInterface Class

The `NodeletManagerInterface` is a wrapper to load, unload the `RobotHwDriverInterface`, that is, to dynamically load a different `nodelet` where a different `RobotHW` performs the operations `read()` and `write()`.

```cpp
class NodeletManagerInterface
{
private:
  ...
  std::shared_ptr<nodelet::Loader>         nodelet_loader_;


public:
  std::map<std::string, cnr::control::ControllerManagerInterface> cmi_;

  NodeletManagerInterface(std::shared_ptr<cnr_logger::TraceLogger> log, const std::string& root_ns, const std::string& nodelet_manager_ns = "/configuration_nodelet_manager");
  ~NodeletManagerInterface()
  {
    nodelet_loader_.reset();
  }

  ros::NodeHandle& getNamespace();
  std::string      loadServiceName();
  std::string      unloadServiceName();
  std::string      listServiceName();

  bool loadRequest(nodelet::NodeletLoad&   msg, std::string& error, const ros::Duration&  watchdog = ros::Duration(0.0));
  bool unloadRequest(nodelet::NodeletUnload& msg, std::string& error, const ros::Duration&  watchdog = ros::Duration(0.0));
  bool listRequest(nodelet::NodeletList&   msg, std::string& error, const ros::Duration&  watchdog = ros::Duration(0.0));

  const std::string& error() const;

  // inline implementation
  bool get_hw_param(ros::NodeHandle &nh, const std::string& hw_name, nodelet::NodeletLoadRequest& request);
  bool list_hw(std::vector<std::string>& hw_names_from_nodelet, const ros::Duration& watchdog);
  bool purge_hw(const ros::Duration& watchdog);
  bool load_hw(const std::string& hw_to_load_name, const ros::Duration& watchdog, bool double_check);
  bool load_hw(const std::vector<std::string>& hw_to_load_names, const ros::Duration& watchdog, bool double_check);
  bool unload_hw(const std::vector<std::string>& hw_to_unload_names, const ros::Duration& watchdog);
};
```