#include <functional>
#include <memory>
#include <stdlib.h>
#include <iostream>

#include <mysql_connection.h>
#include <mysql_driver.h>
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std;

sql::Driver *driver;
sql::Connection *con;
sql::Statement *stmt;
sql::ResultSet *res;

class MysqlNode : public rclcpp::Node
{
public:
  MysqlNode()
  : Node("mysql_node")
  {
    custom_topic_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "custom_topic", 10, std::bind(&MysqlNode::custom_topic_callback, this, _1));

    stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "stop", 10, std::bind(&MysqlNode::stop_callback, this, _1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
      5000ms, std::bind(&MysqlNode::timer_callback, this));
  }

private:
  void custom_topic_callback(const geometry_msgs::msg::Twist & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: x: '%f', z: '%f'", static_cast<double>(msg.linear.x), static_cast<double>(msg.angular.z));
    stmt = con->createStatement();
    string command = "INSERT INTO vel(x, z) VALUES (" + to_string(msg.linear.x) + ", " + to_string(msg.angular.z) + ")";
    stmt->execute(command);
  }

  void stop_callback(const std_msgs::msg::Bool & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard '%d'", msg.data);
    if(msg.data)
    {
      stmt = con->createStatement();
      stmt->execute("TRUNCATE TABLE vel");
    }
  }

  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    string str_x, str_z;

    stmt = con->createStatement();

    // queries the oldest command inserted to the db as per requirement
    res = stmt->executeQuery("SELECT * FROM vel ORDER BY id ASC LIMIT 1");
    while (res->next())
    {
      str_x = res->getString("x");
      str_z = res->getString("z");
    }

    if (str_x != "" || str_z != "")
    {
      message.linear.x = stod(str_x);
      message.linear.y = 0.0;
      message.linear.z = 0.0;
      message.angular.x = 0.0;
      message.angular.y = 0.0;
      message.angular.z = stod(str_z);

      vel_pub_->publish(message);
      RCLCPP_INFO(this->get_logger(), "Published via /cmd_vel with x = %s; z = %s", str_x.c_str(), str_z.c_str());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr custom_topic_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
};

void db_connect()
{
  try {
    // Create a connection
    driver = get_driver_instance();
    con = driver->connect("tcp://mysql:3306", "root", "secret");
    // Connect to the rosdb database
    con->setSchema("rosdb");

    stmt = con->createStatement();
    stmt->execute("CREATE TABLE IF NOT EXISTS vel(id INT AUTO_INCREMENT PRIMARY KEY, x REAL NOT NULL, z REAL NOT NULL)");

  } catch (sql::SQLException &e) {
    cout << "# ERR: SQLException in " << __FILE__;
    cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << endl;
    cout << "# ERR: " << e.what();
    cout << " (MySQL error code: " << e.getErrorCode();
    cout << ", SQLState: " << e.getSQLState() << " )" << endl;
  }
}

int main(int argc, char * argv[])
{
  db_connect();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MysqlNode>());
  rclcpp::shutdown();

  delete res;
  delete stmt;
  delete con;

  return 0;
}
