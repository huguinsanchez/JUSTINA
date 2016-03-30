#include <iostream>
#include <QApplication>
#include <QPushButton>
#include "ros/ros.h"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    QWidget window;
    window.setFixedSize(200, 200);
    QPushButton* button = new QPushButton("Hello world!", &window);
    button->setText("MyText");
    button->setToolTip("The tooltip");
    QFont font("Courier");
    button->setFont(font);
    button->setIcon(QIcon::fromTheme("face-smile"));
    button->setGeometry(100, 100, 80, 30);
    //button.show();
    window.show();
    return app.exec();
    
    std::cout << "INITIALIZING JUSTINA GUI BY MARCOSOFT" << std::endl;
    ros::init(argc, argv, "justina_gui");
    ros::NodeHandle n;
    ros::Rate loop(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}