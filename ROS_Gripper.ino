 #include <ros.h>
    #include <std_msgs/String.h>
    #include <std_msgs/UInt16.h>
    
    #define RELAY1 2
    #define RELAY2 3
    
    ros::NodeHandle node_handle;
    
    std_msgs::String gripper_read_msg;
    std_msgs::UInt16 gripper_msg
    
    void subsscriberCallback(const std_msgs::UInt16& gripper_msg) {
        switch(gripper_msg.data) {
        case 1:
            digitalWrite(RELAY1, HIGH);
            digitalWrite(RELAY2, LOW);
        case 2:
            digitalWrite(RELAY1, LOW);
            digitalWrite(RELAY2, HIGH);
        case 3:
            digitalWrite(RELAY1, LOW);
            digitalWrite(RELAY2, LOW);
        }
    }
    
    ros::Publisher gripper_read_publisher("gripper_toggle_read", &gripper_read_msg);
    ros::Subscriber<std_msgs::UInt16> gripper_subscriber("toggle_gripper", subscriberCallback);
    
    int toggle_gripper = 0;
    
    void setup()
    {
        pinMode(2, OUTPUT);
        pinMode(3, OUTPUT);
        pinMode(4, INPUT);
        pinMode(5, INPUT);
        
        node_handle.initNode();
        node_handle.advertise(gripper_read_publisher);
        node_handle.subscribe(gripper_subscriber);
    }
    
    void loop()
    {
        
        if (digitalRead(4) == 1) {
            toggle_gripper = 1;
        }
        
        if (digitalRead(5) == 1) {
            toggle_gripper = 2;
        }
        
        if (toggle_gripper == 1) {
            gripper_read_msg.data = "Gripper Closed";
        }
        else if (toggle_gripper == 2) {
            gripper_read_msg.data = "Gripper Opened";
        }
        else {
            gripper_read_msg.data = "Gripper state unknown";
        }
        
        gripper_read_publisher.publish(&gripper_read_msg);
        node_handle.spinOnce();
        
        delay(100);
    }
    
