#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLineEdit>
#include <std_msgs/String.h>

namespace carla1s_rviz_plugins
{
    class BehaviorTreePanel: public rviz::Panel
    {
        Q_OBJECT
        public:
            BehaviorTreePanel( QWidget* parent = 0 );

        public Q_SLOTS:

        protected Q_SLOTS:
            void update_button_set();
            void button_1_click_handler();
            void button_2_click_handler();
            void button_3_click_handler();

        protected:
            QPushButton* button_1;
            QPushButton* button_2;
            QPushButton* button_3;

            QLineEdit* mode_label;

            ros::NodeHandle node_handler;
            ros::Publisher demo_pub;
            ros::Subscriber demo_sub;

            void DemoCallback(const std_msgs::String::ConstPtr& msg);
            void update_mode_label(const QString& str);

    };
} // namespace carla1s_rviz_plugins

// 声明该类是一个rivz的插件类
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(carla1s_rviz_plugins::BehaviorTreePanel,rviz::Panel )