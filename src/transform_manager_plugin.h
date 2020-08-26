#ifndef TRANSFORM_MANAGER_TRANSFORM_MANAGER_PLUGIN_H
#define TRANSFORM_MANAGER_TRANSFORM_MANAGER_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_transform_manager_plugin.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>

namespace tf2
{
    class Quaternion;
}

namespace transform_manager
{
    
class TransformManagerPlugin: public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    TransformManagerPlugin();
    
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private slots:
    void onTransformChanged(int index);
    
    void onParentFrameEditingFinished();
    void onTranslationEditingFinished();
    void onQaternionEditingFinished();
    void onAxisAngleEditingFinished();
    void onEulerEditingFinished();
    void onRPYEditingFinished();
    
private:
    void staticTransformCallback(const tf2_msgs::TFMessage & msg);
    void sendUpdate(std::string const &transform);
    void updateGUI(std::string const &transform);
    void updateCurrentQuaternion(tf2::Quaternion const &q);
    std::string ensureTransformExists();
    
    Ui::TransformManagerWidget m_ui;
    QWidget* m_widget;
    ros::Subscriber m_staticTransformSubscriber;
    ros::ServiceClient m_serviceClient;
    
    std::map<std::string, geometry_msgs::TransformStamped> m_transformMap;
    
    //bool m_updating; 
};
    
} // namespace transform_manager

#endif
