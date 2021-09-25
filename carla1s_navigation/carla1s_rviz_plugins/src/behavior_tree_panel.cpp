#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLineEdit>
#include <std_msgs/String.h>

#include "behavior_tree_panel.h"

namespace carla1s_rviz_plugins
{
    BehaviorTreePanel::BehaviorTreePanel(QWidget* parent): rviz::Panel(parent)
    {
        // GUI: 根布局
        QVBoxLayout *layout_root = new QVBoxLayout;

        // GUI: 状态指示器
        QHBoxLayout *layout_mode_statment = new QHBoxLayout;
        // GUI: 状态指示器/静态标签
        layout_mode_statment->addWidget(new QLabel("Mode:"));
        // GUI: 状态指示器/只读输入框
        mode_label = new QLineEdit;
        mode_label->setReadOnly(true);
        layout_mode_statment->addWidget(mode_label);
        // GUI: 应用状态指示器
        layout_root->addLayout(layout_mode_statment);

        // GUI: 按钮组
        QHBoxLayout *layout_btn_set = new QHBoxLayout;
        // GUI: 按钮
        button_1 = new QPushButton("Button 1"); //TODO: 修改按钮名称（左）
        button_2 = new QPushButton("Button 2"); //TODO: 修改按钮名称（中）
        button_3 = new QPushButton("Button 3"); //TODO: 修改按钮名称（右）
        layout_btn_set->addWidget(button_1);
        layout_btn_set->addWidget(button_2);
        layout_btn_set->addWidget(button_3);
        // GUI: 应用按钮组
        layout_root->addLayout(layout_btn_set);

        // GUI: 应用根布局
        setLayout(layout_root);

        // GUI: 建立事件绑定
        connect(button_1,SIGNAL(clicked()),this,SLOT(button_1_click_handler()));
        connect(button_2,SIGNAL(clicked()),this,SLOT(button_2_click_handler()));
        connect(button_3,SIGNAL(clicked()),this,SLOT(button_3_click_handler()));

        // ROS: 初始化Publisher和Subscribe
        // TODO: 修改Topic名称
        demo_pub = node_handler.advertise<std_msgs::String>("/rviz_bt_plugin/demo",1000);
        demo_sub = node_handler.subscribe("/rviz_bt_plugin/demo",1000,&BehaviorTreePanel::DemoCallback,this);
    }

    // 按钮1点击事件处理程序
    void BehaviorTreePanel::button_1_click_handler()
    {
        //TODO：修改消息内容
        std_msgs::String msg;
        msg.data="Button 1 Clicked!";
        demo_pub.publish(msg);
    }

    // 按钮2点击事件处理程序
    void BehaviorTreePanel::button_2_click_handler()
    {
        //TODO：修改消息内容
        std_msgs::String msg;
        msg.data="Button 2 Clicked!";
        demo_pub.publish(msg);
    }

    // 按钮3点击事件处理程序
    void BehaviorTreePanel::button_3_click_handler()
    {
        //TODO：修改消息内容
        std_msgs::String msg;
        msg.data="Button 3 Clicked!";
        demo_pub.publish(msg);
    }

    // ROS Subscribe回调处理程序
    void BehaviorTreePanel::DemoCallback(const std_msgs::String::ConstPtr& msg)
    {
        update_mode_label(msg->data.c_str());
    }

    // 更新Mode展示框内容
    void BehaviorTreePanel::update_mode_label(const QString& str)
    {
        mode_label->setText(str);
    }
} // namespace carla1s_rviz_plugins

// 声明该类是一个rivz的插件类
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(carla1s_rviz_plugins::BehaviorTreePanel,rviz::Panel )
