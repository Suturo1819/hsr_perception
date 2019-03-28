/**
 * Action Server for Object data that the hsr perception module has observed
 * @author Fenja Kollasch
 */

#include <PerceptionActionServer.h>

using namespace suturo_perception_msgs;


PerceptionActionServer::PerceptionActionServer(std::string &name, std::string pipeline, std::string savePath) :
    action_name(name)
    //processManager(nh, savePath)
{
        ROS_INFO("Initializing RoboSherlock...");
        uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");
        resourceManager.setLoggingLevel(uima::LogStream::EnError);
        std::string pipelinePath;
        rs::common::getAEPaths(pipeline, pipelinePath);

        //processManager.init(pipelinePath);
        uima::ErrorInfo errorInfo;
        mongo::client::GlobalInstance instance;
}

PerceptionActionServer::~PerceptionActionServer() {
    //processManager.stop();
    outInfo("ActionServer was killed.");
}

void makeObjectDetectionData(geometry_msgs::PoseStamped pose, rs::Geometry geometry, u_int shape, std::string objClass, float confidence, std::string knownObjClass, float knownObjConfidence, ObjectDetectionData &odd) {
    odd.pose = pose;
    auto boundingBox = geometry.boundingBox();
    odd.width = boundingBox.width();
    odd.height = boundingBox.height();
    odd.depth = boundingBox.depth();
    odd.name = "Object (" + objClass + ")";
    odd.shape = shape;

    odd.obj_class = objClass;
    odd.confidence = confidence;

    odd.known_obj_class = knownObjClass;
    odd.known_obj_confidence = knownObjConfidence;
}


// Todo: Think of a smarter way to transform pose types
void rsPoseToGeoPose(rs::StampedPose pose, geometry_msgs::PoseStamped &geoPose) {
    auto translation = pose.translation.get();
    auto rotation = pose.rotation.get();

    // Pose infos
    geoPose.pose.position.x = translation[0];
    geoPose.pose.position.y = translation[1];
    geoPose.pose.position.z = translation[2];
    geoPose.pose.orientation.x = rotation[0];
    geoPose.pose.orientation.y = rotation[1];
    geoPose.pose.orientation.z = rotation[2];
    geoPose.pose.orientation.w = rotation[3];

    // Header infos
    geoPose.header.frame_id = pose.frame.get();
    geoPose.header.stamp.sec = pose.timestamp.get()/1000000000;
    geoPose.header.stamp.nsec = pose.timestamp.get();
}


void PerceptionActionServer::process(bool visualize, std::vector<ObjectDetectionData> &detection_data) {


   /* processManager.run(visualize);

    uima::CAS* tcas = processManager.engine_.getCas();
    rs::SceneCas cas(*tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD, *cloud_ptr);

    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);
    for (auto &cluster : clusters) {
        getClusterFeatures(cluster, detection_data);
    }*/

}

void PerceptionActionServer::getClusterFeatures(rs::ObjectHypothesis cluster, std::vector<ObjectDetectionData> &data) {

    std::vector<rs::Geometry> geometry;
    cluster.annotations.filter(geometry);

    if(!geometry.empty()) {
        std::vector<rs::PoseAnnotation> poses;
        cluster.annotations.filter(poses);

        std::vector<rs::Classification> classification;
        cluster.annotations.filter(classification);

        std::vector<rs::ClassConfidence> confi;
        cluster.annotations.filter(confi);

        std::vector<rs::Shape> shapes;
        cluster.annotations.filter(shapes);

        geometry_msgs::PoseStamped poseStamped;
        std::string objClass;
        std::string knownObjClass;
        u_int shape = 0;
        float confidence = 0;
        float knownObjConfidence = 0;
        ObjectDetectionData odd;
        if(!poses.empty()) {
            rsPoseToGeoPose(poses[0].world.get(), poseStamped);
        } else {
            ROS_WARN("Warning: No pose information was perceived");
        }
        if(!classification.empty()){
            objClass = classification[0].classname.get();
            knownObjClass = classification[1].classname.get();
            outInfo("OBJCLASSNAME >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>: " << classification[0].classname.get());
            outInfo("OBJCLASSNAME >>>>>>>>>KNOWN>>>>>>>KNOWN>>>>>>>: " << classification[1].classname.get());
        } else {
            ROS_WARN("Warning: No object class was perceived");
        }
        if(!confi.empty()){
            confidence = confi[0].score.get();
            knownObjConfidence = confi[1].score.get();
            outInfo("OBJCLASSCONFI <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<: " << confi[0].score.get());
            outInfo("OBJCLASSCONFI <<<<<<<<<KNOWN<<<<<<<KNOWN<<<<<<: " << confi[1].score.get());
        } else {
            ROS_WARN("Warning: No confidence was perceived");
        }

        makeObjectDetectionData(poseStamped, geometry[0], shape, objClass, confidence, knownObjClass, knownObjConfidence, odd);
        data.push_back(odd);


    } else {
        ROS_WARN("Object Feature detection was unsuccessful. No geometries were recognized for this object.");
    }

}



