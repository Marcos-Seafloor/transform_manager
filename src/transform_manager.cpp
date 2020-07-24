#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>


// Following is from https://github.com/ros/geometry2/blob/noetic-devel/tf2_ros/src/static_transform_broadcaster_program.cpp
bool validateXmlRpcTf(XmlRpc::XmlRpcValue tf_data) {
  // Validate a TF stored in XML RPC format: ensures the appropriate fields
  // exist. Note this does not check data types.
  return tf_data.hasMember("child_frame_id") &&
         tf_data.hasMember("header") &&
         tf_data["header"].hasMember("frame_id") &&
         tf_data.hasMember("transform") &&
         tf_data["transform"].hasMember("translation") &&
         tf_data["transform"]["translation"].hasMember("x") &&
         tf_data["transform"]["translation"].hasMember("y") &&
         tf_data["transform"]["translation"].hasMember("z") &&
         tf_data["transform"].hasMember("rotation") &&
         tf_data["transform"]["rotation"].hasMember("x") &&
         tf_data["transform"]["rotation"].hasMember("y") &&
         tf_data["transform"]["rotation"].hasMember("z") &&
         tf_data["transform"]["rotation"].hasMember("w");
};

class TransformManager
{
public:
    TransformManager(std::vector<std::string> const &transform_params)
    {
        ros::NodeHandle node;
        for(auto param: transform_params)
        {
            XmlRpc::XmlRpcValue transform_value;
            if(node.getParam(param,transform_value))
            {
                if(transform_value.getType() == XmlRpc::XmlRpcValue::TypeArray)
                {
                    for(int i = 0; i < transform_value.size(); i++)
                    {
                        if(validateXmlRpcTf(transform_value[i]))
                            addTransform(transform_value[i]);
                    }
                }
                else if(validateXmlRpcTf(transform_value))
                    addTransform(transform_value);
            }

        }
    }
private:
    void addTransform(XmlRpc::XmlRpcValue const &transform)
    {
        geometry_msgs::TransformStamped t;
        t.header.frame_id = std::string(transform["header"]["frame_id"]);
        t.child_frame_id = std::string(transform["child_frame_id"]);
        t.transform.translation.x = transform["transform"]["translation"]["x"];
        t.transform.translation.y = transform["transform"]["translation"]["y"];
        t.transform.translation.z = transform["transform"]["translation"]["z"];
        t.transform.rotation.x = transform["transform"]["rotation"]["x"];
        t.transform.rotation.y = transform["transform"]["rotation"]["y"];
        t.transform.rotation.z = transform["transform"]["rotation"]["z"];
        t.transform.rotation.w = transform["transform"]["rotation"]["w"];
        m_broadcaster.sendTransform(t);
    }
    
    tf2_ros::StaticTransformBroadcaster m_broadcaster;   
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_manager_node");
    
    std::vector<std::string> transform_params;
    for(int i = 1; i < argc; i++)
        transform_params.push_back(argv[i]);
    
    TransformManager tm(transform_params);
    
    ros::spin();
    
    return 0;
}
