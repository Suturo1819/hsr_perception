/**
 * Action Server for Object data that the hsr perception module has observed
 * @author Fenja Kollasch
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <suturo_perception_msgs/ObjectDetectionData.h>
#include <suturo_perception_msgs/PerceiveAction.h>

// RoboSherlock stuff
#include <rs/flowcontrol/RSAnalysisEngine.h>
#include <rs/scene_cas.h>
#include <rs/utils/common.h>
#include <rs/types/all_types.h>
#include <mongo/client/dbclient.h>

#include "include/PerceptionActionServer.h"

using namespace suturo_perception_msgs;



void makeObjectDetectionData(geometry_msgs::PoseStamped pose, rs::Geometry geometry, u_int shape, std::string objClass, float confidence, ObjectDetectionData &odd) {
    odd.pose = pose;
    auto boundingBox = geometry.boundingBox();
    odd.width = boundingBox.width();
    odd.height = boundingBox.height();
    odd.depth = boundingBox.depth();
    odd.name = "Object (" + objClass + ")";
    odd.shape = shape;

    odd.obj_class = objClass;
    odd.confidence = confidence;
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


void PerceptionActionServer::execPipeline(std::string pipeline) {

    uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");
    resourceManager.setLoggingLevel(uima::LogStream::EnError);
    std::string pipelinePath;
    rs::common::getAEPaths(pipeline, pipelinePath);
    engine.init(pipelinePath, false);
    uima::ErrorInfo errorInfo;
    mongo::client::GlobalInstance instance;
    engine.process();

    uima::CAS* tcas = engine.getCas();
    rs::SceneCas cas(*tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD, *cloud_ptr);

    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);
    for (auto &cluster : clusters) {
        getClusterFeatures(cluster, result.detectionData);
    }
    if(!result.detectionData.empty()) {
        feedback.feedback = "Object Feature detection was successful.";
        server.publishFeedback(feedback);
        server.setSucceeded(result);
    } else {
        feedback.feedback = "No object detection data was perceived. Make sure that there are visible objects in the scene.";
        server.publishFeedback(feedback);
    }
    engine.destroy();

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
        u_int shape = 0;
        float confidence = 0;
        ObjectDetectionData odd;
        if(!poses.empty()) {
            rsPoseToGeoPose(poses[0].world.get(), poseStamped);
        } else {
            feedback.feedback = "Warning: No pose information was perceived";
            server.publishFeedback(feedback);
        }
        if(!classification.empty()){
            objClass = classification[0].classname.get();
        } else {
            feedback.feedback = "Warning: No object class was perceived";
            server.publishFeedback(feedback);
        }
        if(!confi.empty()){
            odd.confidence = confi[0].score.get();
        } else {
            feedback.feedback = "Warning: No confidence was perceived";
            server.publishFeedback(feedback);
        }

        makeObjectDetectionData(poseStamped, geometry[0], shape, objClass, confidence, odd);
        data.push_back(odd);


    } else {
        feedback.feedback = "Object Feature detection was unsuccessful. No geometries were recognized for this object.";
        server.publishFeedback(feedback);
    }

}

void PerceptionActionServer::execute(const PerceiveGoalConstPtr &goal) {
    auto pipeline = goal->pipeline;
    if(pipeline == "table") {
        ROS_INFO("Executing table pipeline");
        execPipeline("hsrb_table");
    } else if(pipeline == "shelf") {
        ROS_INFO("Executing shelf pipeline");
        execPipeline("hsrb_shelf");
    } else {
        feedback.feedback = "There is no pipeline defined for " + pipeline;
        server.publishFeedback(feedback);
    }
}



