#include "filters/mean.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace filters
{
template<>
bool MeanFilter<geometry_msgs::msg::WrenchStamped>::update(const geometry_msgs::msg::WrenchStamped & data_in, geometry_msgs::msg::WrenchStamped & data_out)
{
  // update active row
  if (last_updated_row_ >= number_of_observations_ - 1) {
    last_updated_row_ = 0;
  } else {
    ++last_updated_row_;
  }

  data_storage_->push_back(data_in);

  size_t length = data_storage_->size();

  data_out = geometry_msgs::msg::WrenchStamped();
  for (std::size_t row = 0; row < length; ++row)
  {
    geometry_msgs::msg::WrenchStamped data_row = data_storage_->at(row);
    data_out.wrench.force.x += data_row.wrench.force.x;
    data_out.wrench.force.y += data_row.wrench.force.y;
    data_out.wrench.force.z += data_row.wrench.force.z;
    data_out.wrench.torque.x += data_row.wrench.torque.x;
    data_out.wrench.torque.y += data_row.wrench.torque.y;
    data_out.wrench.torque.z += data_row.wrench.torque.z;
  }

  data_out.wrench.force.x /= length;
  data_out.wrench.force.y /= length;
  data_out.wrench.force.z /= length;
  data_out.wrench.torque.x /= length;
  data_out.wrench.torque.y /= length;
  data_out.wrench.torque.z /= length;

  data_out.header = data_in.header;

  return true;
}
}


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(filters::MeanFilter<geometry_msgs::msg::WrenchStamped>, filters::FilterBase<geometry_msgs::msg::WrenchStamped>)
