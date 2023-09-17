#include "merger_obj.h"

int main(int argc, char * argv[])
{
    // create instance of Merger
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Merger>());
    rclcpp::shutdown();
    return 0;
}