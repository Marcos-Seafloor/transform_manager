#include "transform_manager_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QComboBox>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace transform_manager
{

TransformManagerPlugin::TransformManagerPlugin()
{
    setObjectName("TransformManager");
}

void TransformManagerPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
    m_widget = new QWidget();
    m_ui.setupUi(m_widget);
    
    m_widget->setWindowTitle(m_widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    
    context.addWidget(m_widget);

    connect(m_ui.transformListComboBox, SIGNAL(activated(int)), this, SLOT(onTransformChanged(int)));
    
    connect(m_ui.parentFrameLineEdit, SIGNAL(editingFinished()), this, SLOT(onParentFrameEditingFinished()));
    
    m_staticTransformSubscriber = getNodeHandle().subscribe("/tf_static", 10, &TransformManagerPlugin::staticTransformCallback, this);
}

void TransformManagerPlugin::shutdownPlugin()
{
}

void TransformManagerPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
}

void TransformManagerPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
}


void TransformManagerPlugin::staticTransformCallback(const tf2_msgs::TFMessage & msg)
{
    m_ui.transformListComboBox->clear();
    m_transformMap.clear();
    for(auto t: msg.transforms)
    {
        ROS_INFO_STREAM(t.header.frame_id << " -> " << t.child_frame_id);
        m_ui.transformListComboBox->addItem(std::string(t.child_frame_id).c_str());
        m_transformMap[t.child_frame_id] = t;
    }
}

void TransformManagerPlugin::onTransformChanged(int index)
{
    std::string transform = m_ui.transformListComboBox->itemText(index).toStdString();
    auto i = m_transformMap.find(transform);
    if(i != m_transformMap.end())
    {
        m_ui.parentFrameLineEdit->setText(std::string(i->second.header.frame_id).c_str());
        m_ui.xLineEdit->setText(QString::number(i->second.transform.translation.x));
        m_ui.yLineEdit->setText(QString::number(i->second.transform.translation.y));
        m_ui.zLineEdit->setText(QString::number(i->second.transform.translation.z));
        m_ui.qXLineEdit->setText(QString::number(i->second.transform.rotation.x));
        m_ui.qYLineEdit->setText(QString::number(i->second.transform.rotation.y));
        m_ui.qZLineEdit->setText(QString::number(i->second.transform.rotation.z));
        m_ui.qWLineEdit->setText(QString::number(i->second.transform.rotation.w));
        
        tf2::Quaternion q;
        tf2::convert(i->second.transform.rotation, q);
        m_ui.aaAngleLineEdit->setText(QString::number(q.getAngle()*180/M_PI));
        m_ui.aaXLineEdit->setText(QString::number(q.getAxis()[0]));
        m_ui.aaYLineEdit->setText(QString::number(q.getAxis()[1]));
        m_ui.aaZLineEdit->setText(QString::number(q.getAxis()[2]));
        
        tf2::Matrix3x3 mat;
        mat.setRotation(q);
        
        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);
        m_ui.eYawLineEdit->setText(QString::number(yaw*180/M_PI));
        m_ui.ePitchLineEdit->setText(QString::number(pitch*180/M_PI));
        m_ui.eRollLineEdit->setText(QString::number(roll*180/M_PI));
        
        mat.getRPY(roll, pitch, yaw);
        m_ui.rpyRollLineEdit->setText(QString::number(roll*180/M_PI));
        m_ui.rpyPitchLineEdit->setText(QString::number(pitch*180/M_PI));
        m_ui.rpyYawLineEdit->setText(QString::number(yaw*180/M_PI));
    }
   
}

void TransformManagerPlugin::onParentFrameEditingFinished()
{
    std::cerr << "parent frame: " << m_ui.parentFrameLineEdit->text().toStdString() << std::endl;
}

void TransformManagerPlugin::onTranslationEditingFinished()
{
    std::cerr << "translation: " << m_ui.xLineEdit->text().toStdString() << ", " << m_ui.yLineEdit->text().toStdString() << ", " << m_ui.zLineEdit->text().toStdString() << std::endl;
}

void TransformManagerPlugin::onQaternionEditingFinished()
{
    std::cerr << "quaternion: " << m_ui.qXLineEdit->text().toStdString() << ", " << m_ui.qYLineEdit->text().toStdString() << ", " << m_ui.qZLineEdit->text().toStdString() << ", " << m_ui.qWLineEdit->text().toStdString() << std::endl;
}

void TransformManagerPlugin::onAxisAngleEditingFinished()
{
    
}

void TransformManagerPlugin::onEulerEditingFinished()
{
}

void TransformManagerPlugin::onRPYEditingFinished()
{
}

} // namespace transform_manager

PLUGINLIB_EXPORT_CLASS(transform_manager::TransformManagerPlugin, rqt_gui_cpp::Plugin)
