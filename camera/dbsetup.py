import mysql.connector

mydb = mysql.connector.connect(
  host="localhost",
  user="emotioneering",
  password="password",
  database="history"
)

mycursor = mydb.cursor()

#mycursor.execute("CREATE DATABASE history")

#mycursor.execute("SHOW DATABASES")

#for x in mycursor:
  #print(x)

mycursor.execute("CREATE TABLE history (id int AUTO_INCREMENT PRIMARY KEY, time DATETIME, enterCount smallint unsigned, exitCount smallint unsigned)")

mycursor.execute("SHOW TABLES")

for x in mycursor:
  print(x)