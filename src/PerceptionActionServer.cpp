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

#include "include/PerceptionActionServer.h"

using namespace suturo_perception_msgs;


u_int shape_map(std::string shape) {
    if (shape == "round" || shape == "cylinder") {
        return ObjectDetectionData::CYLINDER;
    } else if (shape == "box") {
        return ObjectDetectionData::BOX;
    }
    return ObjectDetectionData::MISC;
}

void makeObjectDetectionData(geometry_msgs::PoseStamped pose, rs::Geometry geometry, rs::Shape shape, ObjectDetectionData &odd) {
    odd.shape = shape_map(shape.shape());
    odd.pose = pose;
    auto boundingBox = geometry.boundingBox();
    odd.width = boundingBox.width();
    odd.height = boundingBox.height();
    odd.depth = boundingBox.depth();
    odd.name = "Object (" + pose.header.frame_id + ")";
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
    std::string pipelinePath;
    rs::common::getAEPaths(pipeline, pipelinePath);
    engine.init(pipelinePath, false);
    engine.process();

    uima::CAS* tcas = engine.getCas();
    rs::SceneCas cas(*tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    cas.get(VIEW_CLOUD, *cloud_ptr);

    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);
    for (auto &cluster : clusters) {
        getClusterFeatures(cluster);
    }

}

void PerceptionActionServer::getClusterFeatures(rs::ObjectHypothesis cluster) {

    std::vector<rs::Shape> shapes;
    cluster.annotations.filter(shapes);
    std::vector<rs::Geometry> geometry;
    cluster.annotations.filter(geometry);

    if(!shapes.empty() && !geometry.empty()) {
        std::vector<rs::PoseAnnotation> poses;
        cluster.annotations.filter(poses);
        for (auto &pose : poses) {
            ObjectDetectionData odd;
            geometry_msgs::PoseStamped poseStamped;
            rsPoseToGeoPose(pose.world.get(), poseStamped);
            makeObjectDetectionData(poseStamped, geometry[0], shapes[0], odd);

            result.detectionData = odd;
            server.setSucceeded(result);
            feedback.feedback = "Object Feature detection was successful.";
            server.publishFeedback(feedback);
        }
    } else {
        feedback.feedback = "Object Feature detection was unsuccessful. It seems like no shapes/poses were recognized.";
        server.publishFeedback(feedback);
    }

}

void PerceptionActionServer::execute(const PerceiveGoalConstPtr &goal) {
    auto pipeline = goal->pipeline;
    if(pipeline == "table") {
        execPipeline("hsrb_table");
    } else if(pipeline == "shelf") {
        execPipeline("hsrb_shelf");
    } else {
        feedback.feedback = "There is no pipeline defined for " + pipeline;
        server.publishFeedback(feedback);
    }
}



