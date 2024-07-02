# Week 2 Web and NLP Task 1: Building a Web Page to Control Robot Direction and Connecting to a Database

## Table of Contents
1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Setting Up the Web Page](#setting-up-the-web-page)
4. [Creating the Database](#creating-the-database)
5. [Connecting the Web Page to the Database](#connecting-the-web-page-to-the-database)
6. [Controlling the Robot](#controlling-the-robot)
7. [Conclusion](#conclusion)

## Introduction
This task involves creating a web page that allows users to control the direction of a robot and record these directions in a database. This exercise will help you understand web development basics and how to interface web applications with databases.

## Prerequisites
- Basic understanding of HTML, CSS, and JavaScript
- Knowledge of PHP and MySQL
- XAMPP installed on your machine

## Setting Up the Web Page

### Step 1: Create the HTML File
Create an `index.html` file with the following content and place it in the `htdocs` directory of your XAMPP installation:
```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Control</title>
</head>
<body>
    <h1>Robot Control Interface</h1>
    <button onclick="sendCommand('forward')">Forward</button>
    <button onclick="sendCommand('left')">Left</button>
    <button onclick="sendCommand('right')">Right</button>
    <button onclick="sendCommand('backward')">Backward</button>

    <script>
        function sendCommand(direction) {
            fetch('command.php', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ direction: direction })
            });
        }
    </script>
</body>
</html>
```

### Step 2: Set Up the Backend
Create a `command.php` file with the following content and place it in the `htdocs` directory of your XAMPP installation:
```php
<?php
$servername = "localhost";
$username = "root";
$password = "";
$dbname = "robot_commands";

// Create connection
$conn = new mysqli($servername, $username, $password, $dbname);

// Check connection
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
}

$input = json_decode(file_get_contents('php://input'), true);
$direction = $input['direction'];

$sql = "INSERT INTO commands (direction) VALUES ('$direction')";

if ($conn->query($sql) === TRUE) {
    echo json_encode(["status" => "success"]);
} else {
    echo json_encode(["status" => "error", "message" => $conn->error]);
}

$conn->close();
?>
```

## Creating the Database
Create a MySQL database using phpMyAdmin or the MySQL command line.

### Step 1: Create the Database
```sql
CREATE DATABASE robot_commands;
```

### Step 2: Create the Table
```sql
USE robot_commands;

CREATE TABLE commands (
    id INT(6) UNSIGNED AUTO_INCREMENT PRIMARY KEY,
    direction VARCHAR(30) NOT NULL,
    reg_date TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);
```

## Connecting the Web Page to the Database
The `sendCommand` function in the `index.html` file sends POST requests to the `command.php` endpoint, which stores the commands in the MySQL database.

## Controlling the Robot
To control the robot, you'll need to extend the backend to interface with your ROS setup. Here's an example of how you could publish commands to a ROS topic using a Python script executed from PHP:

### Step 1: Update `command.php` to Call a Python Script
Update the `command.php`:
```php
<?php
$servername = "localhost";
$username = "root";
$password = "";
$dbname = "robot_commands";

// Create connection
$conn = new mysqli($servername, $username, $password, $dbname);

// Check connection
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
}

$input = json_decode(file_get_contents('php://input'), true);
$direction = $input['direction'];

$sql = "INSERT INTO commands (direction) VALUES ('$direction')";

if ($conn->query($sql) === TRUE) {
    // Call the Python script to publish the command to ROS
    exec("python3 /path/to/your/script.py $direction");
    echo json_encode(["status" => "success"]);
} else {
    echo json_encode(["status" => "error", "message" => $conn->error]);
}

$conn->close();
?>
```

### Step 2: Create the Python Script
Create a `script.py` file with the following content:
```python
import sys
import rospy
from geometry_msgs.msg import Twist

def publish_command(direction):
    rospy.init_node('web_interface', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    if direction == 'forward':
        twist.linear.x = 1.0
    elif direction == 'backward':
        twist.linear.x = -1.0
    elif direction == 'left':
        twist.angular.z = 1.0
    elif direction == 'right':
        twist.angular.z = -1.0
    pub.publish(twist)

if __name__ == '__main__':
    direction = sys.argv[1]
    publish_command(direction)
```

## Conclusion
This document covered the steps to create a web page for controlling a robot's direction and storing the commands in a database using XAMPP. By following these instructions, you should have a functional web interface that can send commands to your robot and log them in a database.
