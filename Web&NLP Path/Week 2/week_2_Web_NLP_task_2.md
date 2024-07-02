
# Week 2 Web and NLP Task 2: Creating a Web Page to Display Stored Data (Last Value)

## Table of Contents
1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Setting Up the Web Page](#setting-up-the-web-page)
4. [Creating the Backend to Fetch Data](#creating-the-backend-to-fetch-data)
5. [Displaying Data on the Web Page](#displaying-data-on-the-web-page)
6. [Conclusion](#conclusion)

## Introduction
This task involves creating a web page that displays the last recorded command direction from the database. This will help you understand how to fetch and display data from a database using a web application.

## Prerequisites
- Basic understanding of HTML, CSS, and JavaScript
- Knowledge of PHP and MySQL
- XAMPP installed on your machine

## Setting Up the Web Page

### Step 1: Create the HTML File
Create an `display.html` file with the following content and place it in the `htdocs` directory of your XAMPP installation:
```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Command Display</title>
</head>
<body>
    <h1>Last Robot Command</h1>
    <div id="lastCommand">Loading...</div>

    <script>
        function fetchLastCommand() {
            fetch('fetch_command.php')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('lastCommand').innerText = data.direction;
                })
                .catch(error => {
                    console.error('Error fetching command:', error);
                });
        }

        window.onload = fetchLastCommand;
    </script>
</body>
</html>
```

## Creating the Backend to Fetch Data
Create a `fetch_command.php` file with the following content and place it in the `htdocs` directory of your XAMPP installation:
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

$sql = "SELECT direction FROM commands ORDER BY id DESC LIMIT 1";
$result = $conn->query($sql);

if ($result->num_rows > 0) {
    $row = $result->fetch_assoc();
    echo json_encode(["direction" => $row["direction"]]);
} else {
    echo json_encode(["direction" => "No commands found"]);
}

$conn->close();
?>
```

## Displaying Data on the Web Page
The JavaScript function `fetchLastCommand` in the `display.html` file sends a GET request to the `fetch_command.php` endpoint, which retrieves the last command from the MySQL database and returns it as a JSON response. This data is then displayed in the `lastCommand` div.

## Conclusion
This document covered the steps to create a web page that fetches and displays the last recorded command direction from a database using XAMPP. By following these instructions, you should have a functional web interface that displays the most recent command issued to your robot.
