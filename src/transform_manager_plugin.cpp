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
    
    connect(m_ui.xLineEdit, SIGNAL(editingFinished()), this, SLOT(onTranslationEditingFinished()));
    connect(m_ui.yLineEdit, SIGNAL(editingFinished()), this, SLOT(onTranslationEditingFinished()));
    connect(m_ui.zLineEdit, SIGNAL(editingFinished()), this, SLOT(onTranslationEditingFinished()));
    
    connect(m_ui.qXLineEdit, SIGNAL(editingFinished()), this, SLOT(onQaternionEditingFinished()));
    connect(m_ui.qYLineEdit, SIGNAL(editingFinished()), this, SLOT(onQaternionEditingFinished()));
    connect(m_ui.qZLineEdit, SIGNAL(editingFinished()), this, SLOT(onQaternionEditingFinished()));
    connect(m_ui.qWLineEdit, SIGNAL(editingFinished()), this, SLOT(onQaternionEditingFinished()));

    connect(m_ui.aaAngleLineEdit, SIGNAL(editingFinished()), this, SLOT(onAxisAngleEditingFinished()));
    connect(m_ui.aaXLineEdit, SIGNAL(editingFinished()), this, SLOT(onAxisAngleEditingFinished()));
    connect(m_ui.aaYLineEdit, SIGNAL(editingFinished()), this, SLOT(onAxisAngleEditingFinished()));
    connect(m_ui.aaZLineEdit, SIGNAL(editingFinished()), this, SLOT(onAxisAngleEditingFinished()));

    connect(m_ui.eYawLineEdit, SIGNAL(editingFinished()), this, SLOT(onEulerEditingFinished()));
    connect(m_ui.ePitchLineEdit, SIGNAL(editingFinished()), this, SLOT(onEulerEditingFinished()));
    connect(m_ui.eRollLineEdit, SIGNAL(editingFinished()), this, SLOT(onEulerEditingFinished()));

    connect(m_ui.rpyRollLineEdit, SIGNAL(editingFinished()), this, SLOT(onRPYEditingFinished()));
    connect(m_ui.rpyPitchLineEdit, SIGNAL(editingFinished()), this, SLOT(onRPYEditingFinished()));
    connect(m_ui.rpyYawLineEdit, SIGNAL(editingFinished()), this, SLOT(onRPYEditingFinished()));
    
    m_staticTransformSubscriber = getNodeHandle().subscribe("/tf_static", 10, &TransformManagerPlugin::staticTransformCallback, this);
}

void TransformManagerPlugin::shutdownPlugin()
{ 
    m_staticTransformSubscriber.shutdown();
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
    updateGUI(m_ui.transformListComboBox->itemText(index).toStdString());
    
}

void TransformManagerPlugin::updateGUI(std::string const &transform)
{
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

std::string  TransformManagerPlugin::ensureTransformExists()
{
    std::string transform = m_ui.transformListComboBox->currentText().toStdString();
    if(m_transformMap.find(transform) == m_transformMap.end())
    {
        m_transformMap[transform].header.frame_id == m_ui.parentFrameLineEdit->text().toStdString();
        m_transformMap[transform].child_frame_id == transform;
        m_transformMap[transform].transform.translation.x = m_ui.xLineEdit->text().toDouble();
        m_transformMap[transform].transform.translation.y = m_ui.yLineEdit->text().toDouble();
        m_transformMap[transform].transform.translation.z = m_ui.zLineEdit->text().toDouble();
        m_transformMap[transform].transform.rotation.x = m_ui.qXLineEdit->text().toDouble();
        m_transformMap[transform].transform.rotation.y = m_ui.qYLineEdit->text().toDouble();
        m_transformMap[transform].transform.rotation.z = m_ui.qZLineEdit->text().toDouble();
        m_transformMap[transform].transform.rotation.w = m_ui.qWLineEdit->text().toDouble();
    }
    return transform;
}


void TransformManagerPlugin::updateCurrentQuaternion(tf2::Quaternion const &q)
{
    tf2::Quaternion nq = q.normalized();
    
    std::string transform = ensureTransformExists();
    
    m_transformMap[transform].transform.rotation = tf2::toMsg(nq);
    
    updateGUI(transform);
}

void TransformManagerPlugin::onParentFrameEditingFinished()
{
    std::string transform = ensureTransformExists();
    m_transformMap[transform].header.frame_id = m_ui.parentFrameLineEdit->text().toStdString();
}

void TransformManagerPlugin::onTranslationEditingFinished()
{
    std::string transform = ensureTransformExists();
    m_transformMap[transform].transform.translation.x = m_ui.xLineEdit->text().toDouble();
    m_transformMap[transform].transform.translation.y = m_ui.yLineEdit->text().toDouble();
    m_transformMap[transform].transform.translation.z = m_ui.zLineEdit->text().toDouble();
}

void TransformManagerPlugin::onQaternionEditingFinished()
{
    updateCurrentQuaternion(tf2::Quaternion(m_ui.qXLineEdit->text().toDouble(), m_ui.qYLineEdit->text().toDouble(), m_ui.qZLineEdit->text().toDouble(), m_ui.qWLineEdit->text().toDouble()));
}

void TransformManagerPlugin::onAxisAngleEditingFinished()
{
    updateCurrentQuaternion(tf2::Quaternion(tf2::Vector3(m_ui.aaXLineEdit->text().toDouble(), m_ui.aaYLineEdit->text().toDouble(), m_ui.aaZLineEdit->text().toDouble()), m_ui.aaAngleLineEdit->text().toDouble()*M_PI/180.0));
}

void TransformManagerPlugin::onEulerEditingFinished()
{
    tf2::Quaternion q;
    q.setEuler(m_ui.eYawLineEdit->text().toDouble()*M_PI/180.0,m_ui.ePitchLineEdit->text().toDouble()*M_PI/180.0,m_ui.eRollLineEdit->text().toDouble()*M_PI/180.0);
    updateCurrentQuaternion(q);
}

void TransformManagerPlugin::onRPYEditingFinished()
{
    tf2::Quaternion q;
    q.setRPY(m_ui.rpyRollLineEdit->text().toDouble()*M_PI/180.0, m_ui.rpyPitchLineEdit->text().toDouble()*M_PI/180.0, m_ui.rpyYawLineEdit->text().toDouble()*M_PI/180.0);
    updateCurrentQuaternion(q);
}

} // namespace transform_manager

PLUGINLIB_EXPORT_CLASS(transform_manager::TransformManagerPlugin, rqt_gui_cpp::Plugin)
