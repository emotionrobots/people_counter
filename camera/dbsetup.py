import mysql.connector

mydb = mysql.connector.connect(
  host="localhost",
  user="emotioneering",
  password="password"
)

mycursor = mydb.cursor()

mycursor.execute("CREATE DATABASE history")

mycursor.execute("SHOW DATABASES")

for x in mycursor:
  print(x)