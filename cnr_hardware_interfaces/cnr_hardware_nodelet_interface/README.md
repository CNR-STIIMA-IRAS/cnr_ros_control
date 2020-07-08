# Hardware Nodelet Interface

The package provides two class to ease the configuration and the life-cycle management of the `nodelet`.

## RobotHwNodelet Class

1. `cnr_hardware_nodelet_interface::RobotHwNodelet : public nodelet::Nodelet`  embeds a `RobotHW` inside a `nodelet`. Basically, the Class implements the `onInit` function inherited from the `public nodelet::Nodelet`

```cpp
void RobotHwNodelet::onInit()
{
  m_hw_namespace = getPrivateNodeHandle().getNamespace();
  m_hw_name      = extractRobotName(m_hw_namespace);
  m_updater.setHardwareID(m_hw_name);
  m_logger.reset(new cnr_logger::TraceLogger("NL_" + m_hw_name, m_hw_namespace));

  try
  {
    CNR_TRACE_START(*m_logger);
    if (enterOnInit() && doOnInit() && exitOnInit())
    {
      m_stop_update_thread = m_stop_diagnostic_thread = false;
      CNR_RETURN_OK(*m_logger, void());
    }
  }
  catch (std::exception& e)
  {
    CNR_RETURN_NOTOK(*m_logger, void(), m_hw_name + ": ExitOnInit failed. Exception caught: " + std::string(e.what()));
  }
  catch (...)
  {
    CNR_RETURN_NOTOK(*m_logger, void(), m_hw_name + ": ExitOnInit failed. UNhandled Exception");
  }
}
```

The implementation is split in three sub-methods `enterOnInit()`, `doOnInit()` and `exitOnInit()`, so the user can derive and modify `doOnInit()` preserving all the functionalities provided by the class.

The `RobotHwNodelet::enterOnInit()` prepares the classes, the diagnosis module, check the sampling period parameters, and create the `plugin` loader of the `RobotHw` class

```cpp
bool RobotHwNodelet::enterOnInit()
{
  CNR_TRACE_START(*m_logger);
  m_stop_update_thread = m_stop_diagnostic_thread = false;

  // getNodeHandle(), parent nodelet handle (typically "/")  --- getPrivateNodeHandle() -> /nodelet_name/
  m_nh     = getNodeHandle();
  m_hw_nh  = getPrivateNodeHandle();

  double sampling_period = 0.001;
  if (!m_hw_nh.getParam("sampling_period", sampling_period))
  {
    CNR_WARN(*m_logger, m_hw_namespace + "/sampling_period' does not exist, set equal to 0.001");
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

  CNR_RETURN_TRUE(*m_logger);
}
```

The `RobotHwNodelet::doOnInit()` method creates the `RobtHw` objects exploiting the `plugin` loader. The function can be inherited for specific implementation. The `RobotHw` needs an empty Constructor.

```cpp
bool RobotHwNodelet::doOnInit()
{
  CNR_TRACE_START(*m_logger);
  std::string robot_type;
  try
  {
    if (!m_hw_nh.getParam("type", robot_type))
    {
      CNR_FATAL(*m_logger, "The param '" << m_hw_nh.getNamespace() << "/type' is missing! Abort.");
      CNR_RETURN_FALSE(*m_logger);
    }
    m_robot_hw_plugin_loader->loadLibraryForClass(robot_type);
    m_hw = m_robot_hw_plugin_loader->createInstance(robot_type);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = m_robot_hw_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    CNR_ERROR(*m_logger, "Exception while loading '" << robot_type << "': " << ex.what() << "\n Available plugins: " << ss.str());
  }
  catch (std::exception& e)
  {
    CNR_RETURN_FALSE(*m_logger, "OnInit of FakeRobotHw failed! what: " + std::string(e.what()));
  }
  CNR_RETURN_TRUE(*m_logger);
}
```

Finally, `RobotHwNodelet::exitOnInit()` creates two running threads: a diagnostic thread, and the control thread. Furthermore, the `ControllerManager` [(ros_control)](http://wiki.ros.org/ros_control) is created, in order to allow the dynamic loading/unloading of the controller. 

```cpp
bool RobotHwNodelet::exitOnInit()
{
  CNR_TRACE_START(*m_logger);
  CNR_DEBUG(*m_logger, "Triggering the start of the main thread");

  if (m_hw == nullptr)
  {
    CNR_RETURN_FALSE(*m_logger, "The RobotHw has not been properly initialized in the doOnInit() function. Abort.");
  }

  if (!m_hw->init(m_nh, m_hw_nh))
  {
    CNR_RETURN_FALSE(*m_logger, "The RobotHw '" + m_hw_name + "'Initialization failed. Abort.");
  }

  m_cm.reset(new controller_manager::ControllerManager(m_hw.get(), m_hw_nh));

  m_update_thread_state = ON_INIT;
  ros::Time start       = ros::Time::now();
  m_update_thread       = std::thread(&cnr_hardware_nodelet_interface::RobotHwNodelet::controlUpdateThread, this);
  while (m_update_thread_state != RUNNING)
  {
    if (m_update_thread_state == ON_ERROR)
    {
      CNR_RETURN_FALSE(*m_logger, "The update thread is in Error. Abort.");
    }
    if (m_update_thread_state == EXPIRED)
    {
      CNR_RETURN_FALSE(*m_logger, "The update thread is expired. Something went wrong during the start. Abort.");
    }
    if ((ros::Time::now() - start).toSec() > 10.0)
    {
      CNR_RETURN_FALSE(*m_logger, "Timeout Expired. The update thread did not yet started. Abort.");
    }
    ros::Duration(0.05).sleep();
  }

  m_diagnostics_thread_state = ON_INIT;
  start                      = ros::Time::now();
  m_diagnostics_thread       = std::thread(&cnr_hardware_nodelet_interface::RobotHwNodelet::diagnosticsThread, this);
  while (m_diagnostics_thread_state != RUNNING)
  {
    if (m_diagnostics_thread_state == ON_ERROR)
    {
      CNR_RETURN_FALSE(*m_logger, "Main thread in ERROR");
    }
    if ((ros::Time::now() - start).toSec() > 10.0)
    {
      CNR_RETURN_FALSE(*m_logger, "Timeout Exipred. Main Thread did not started yet. Abort.");
    }
    ros::Duration(0.05).sleep();
  }
  CNR_DEBUG(*m_logger, "The main is running");
  CNR_RETURN_TRUE(*m_logger);
}
```
