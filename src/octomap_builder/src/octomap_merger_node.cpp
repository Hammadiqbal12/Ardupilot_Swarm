// file: src/octomap_merger_node.cpp

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

class OctomapMerger : public rclcpp::Node
{
public:
  OctomapMerger()
  : rclcpp::Node("octomap_merger")
  {
    // Parameters
    this->declare_parameter<int>("vehicle_count", 2);
    this->declare_parameter<std::string>("vehicle_prefix", "/iris");
    this->declare_parameter<std::string>("octomap_topic_suffix", "/octomap_full");
    this->declare_parameter<double>("resolution", 0.25);
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("publish_rate", 2.0);  // Hz

    vehicle_count_ = this->get_parameter("vehicle_count").as_int();
    vehicle_prefix_ = this->get_parameter("vehicle_prefix").as_string();
    octomap_topic_suffix_ = this->get_parameter("octomap_topic_suffix").as_string();
    resolution_ = this->get_parameter("resolution").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    if (vehicle_count_ <= 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "vehicle_count is %d, setting to 1", vehicle_count_);
      vehicle_count_ = 1;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "OctomapMerger: vehicle_count=%d, prefix='%s', suffix='%s'",
      vehicle_count_, vehicle_prefix_.c_str(), octomap_topic_suffix_.c_str());

    // Create an empty global tree (will be rebuilt on each publish)
    global_tree_ = std::make_shared<octomap::OcTree>(resolution_);

    // Create subscriptions for each irisX/octomap_full
    for (int i = 1; i <= vehicle_count_; ++i) {
      std::string topic =
        vehicle_prefix_ + std::to_string(i) + octomap_topic_suffix_;

      auto sub = this->create_subscription<octomap_msgs::msg::Octomap>(
        topic,
        rclcpp::QoS(1).best_effort(),
        [this, topic](const octomap_msgs::msg::Octomap::SharedPtr msg) {
          this->octomapCallback(msg, topic);
        }
      );

      subs_.push_back(sub);
      RCLCPP_INFO(this->get_logger(), "Subscribed to %s", topic.c_str());
    }

    // QoS compatible with latched-style map subscribers
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();   // <--- important

    pub_full_ = this->create_publisher<octomap_msgs::msg::Octomap>(
      "/global_octomap_full",
      map_qos
    );

    // Timer: rebuild + publish merged map periodically
    using namespace std::chrono_literals;
    auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&OctomapMerger::rebuildAndPublishGlobalMap, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "OctomapMerger initialized. Publishing merged map on /global_octomap_full");
  }

private:
  // Store latest map for each topic (full map, incl. free & occupied)
  void octomapCallback(
    const octomap_msgs::msg::Octomap::SharedPtr msg,
    const std::string & topic_name)
  {
    // Convert incoming msg to abstract octomap
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(
      octomap_msgs::msgToMap(*msg));

    if (!abstract_tree) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to convert Octomap from topic %s", topic_name.c_str());
      return;
    }

    auto * tree_raw = dynamic_cast<octomap::OcTree *>(abstract_tree.release());
    if (!tree_raw) {
      RCLCPP_WARN(
        this->get_logger(),
        "Incoming map from %s is not OcTree", topic_name.c_str());
      return;
    }

    // Take ownership in shared_ptr
    std::shared_ptr<octomap::OcTree> tree(tree_raw);

    std::lock_guard<std::mutex> lock(mutex_);
    latest_trees_[topic_name] = tree;

    if (received_topics_.insert(topic_name).second) {
      RCLCPP_INFO(this->get_logger(), "Receiving maps from %s", topic_name.c_str());
    }
  }

  void rebuildAndPublishGlobalMap()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (latest_trees_.empty()) {
      return;
    }

    // Fresh global tree every cycle
    global_tree_ = std::make_shared<octomap::OcTree>(resolution_);

    // Merge all latest trees
    for (const auto & kv : latest_trees_) {
      const auto & tree = kv.second;

      if (!tree) {
        continue;
      }

      // Iterate over all leaf nodes of this tree
      for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
        const double x = it.getX();
        const double y = it.getY();
        const double z = it.getZ();

        const bool occ = tree->isNodeOccupied(*it);
        const double node_value = it->getValue();

        // Look up current global node at this coordinate (if any)
        octomap::OcTreeNode * gnode = global_tree_->search(x, y, z);

        if (!gnode || (occ && global_tree_->isNodeOccupied(gnode))) {
          // Unknown cells simply take the incoming measurement. If the global map already
          // considers this cell free and the incoming map says occupied we skip it so we
          // don't reintroduce stale obstacles.
          global_tree_->setNodeValue(x, y, z, node_value);
          continue;
        }

        if (!occ) {
          // Free observations override anything we have stored for that cell.
          global_tree_->setNodeValue(x, y, z, node_value);
        }
      }
    }

    // Recompute inner occupancies
    global_tree_->updateInnerOccupancy();

    // Publish global map
    octomap_msgs::msg::Octomap msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = this->now();

    if (!octomap_msgs::fullMapToMsg(*global_tree_, msg)) {
      RCLCPP_WARN(this->get_logger(), "Failed to convert global OcTree to Octomap msg");
      return;
    }

    pub_full_->publish(msg);
  }


  // Parameters
  int vehicle_count_;
  std::string vehicle_prefix_;
  std::string octomap_topic_suffix_;
  double resolution_;
  std::string frame_id_;

  // Latest per-drone maps
  std::unordered_map<std::string, std::shared_ptr<octomap::OcTree>> latest_trees_;
  std::unordered_set<std::string> received_topics_;

  // Global merged tree
  std::shared_ptr<octomap::OcTree> global_tree_;
  std::mutex mutex_;

  // Subscriptions and publisher
  std::vector<rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr> subs_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_full_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OctomapMerger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
