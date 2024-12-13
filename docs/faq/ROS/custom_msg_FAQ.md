# FAQ: How to Add Custom Messages in ROS

* Yutian (Tim) Fan
* Dec 13 2024
* ROS version: Noetic

## Advantages of Custom Messages
- Enable custom communication tailored to specific use cases.
- Improve code clarity by organizing data into structured messages.
- Reduce the need for parsing generic messages in your nodes.
- Facilitate efficient collaboration by defining clear data contracts between nodes.

---

## Steps to Create a Custom Message

### 1. Create the .msg Files
- Inside your package directory, create a `msg` folder to store the custom .msg files.
- Define your message file using the format:
  ```
  dependency/data_type name
  ```
- Example: For a message representing a frontier:
  ```
  geometry_msgs/Point centroid
  int32 size
  ```
### 2. Edit CMakeLists.txt
- In the `find_package()` section, add `message_generation`:

  ```  
  find_package(catkin REQUIRED COMPONENTS
    ...
    message_generation
    ...
  )
  ```
- Uncomment `add_message_files()` and specify your .msg files:

  ```  
  add_message_files(
    FILES
    Frontier.msg
    FrontierList.msg
    PathList.msg
  )
  ```
- Uncomment `generate_messages()` and include `std_msgs` under dependencies:

  ```
  generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
  )
  ```

### 3. Edit package.xml
Add the necessary dependencies for generating and using custom messages:

  ```
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
  ```

### 4. Build Your Workspace
- Navigate to your workspace directory:

  ```
  cd ~/catkin_ws
  ```
- Run `catkin_make` to compile your workspace and generate the custom message files.
