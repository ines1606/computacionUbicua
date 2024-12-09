
-- Create the User table
CREATE TABLE User (
    uid INT PRIMARY KEY,
    name VARCHAR(100),
    DateOfBirth DATE
);

-- Create the Guardian table
CREATE TABLE Guardian (
    gid INT PRIMARY KEY,
    name VARCHAR(100)
);

-- Create the GuardianContactInfo table
CREATE TABLE GuardianContactInfo (
    gid INT,
    telephone VARCHAR(15),
    street VARCHAR(255),
    number INT,
    city VARCHAR(100),
    PRIMARY KEY (gid, telephone),
    FOREIGN KEY (gid) REFERENCES Guardian(gid)
);

-- Create the Device table
CREATE TABLE Device (
    MAC VARCHAR(17) PRIMARY KEY,
    name VARCHAR(100)
);

-- Create the Health table
CREATE TABLE Health (
    timestamp DATETIME,
    MAC VARCHAR(17),
    HeartRate INT,
    SPO2 DECIMAL(5, 2),
    PRIMARY KEY (timestamp, MAC),
    FOREIGN KEY (MAC) REFERENCES Device(MAC)
);

-- Create the Activity table
CREATE TABLE Activity (
    timestamp DATETIME,
    MAC VARCHAR(17),
    AccelerationX DECIMAL(10, 5),
    AccelerationY DECIMAL(10, 5),
    AccelerationZ DECIMAL(10, 5),
    GyroscopeX DECIMAL(10, 5),
    GyroscopeY DECIMAL(10, 5),
    GyroscopeZ DECIMAL(10, 5),
    PRIMARY KEY (timestamp, MAC),
    FOREIGN KEY (MAC) REFERENCES Device(MAC)
);

-- Create the Location table
CREATE TABLE Location (
    timestamp DATETIME,
    MAC VARCHAR(17),
    longitude DECIMAL(9, 6),
    latitude DECIMAL(9, 6),
    PRIMARY KEY (timestamp, MAC),
    FOREIGN KEY (MAC) REFERENCES Device(MAC)
);

-- Create the Alert table
CREATE TABLE Alert (
    timestamp DATETIME,
    MAC VARCHAR(17),
    AlertType VARCHAR(50),
    message TEXT,
    PRIMARY KEY (timestamp, MAC),
    FOREIGN KEY (MAC) REFERENCES Device(MAC)
);

CREATE TABLE GuardedBy (
    uid INT,
    gid INT,
    PRIMARY KEY (uid, gid),
    FOREIGN KEY (uid) REFERENCES User(uid),
    FOREIGN KEY (gid) REFERENCES Guardian(gid)
);
