import mysql.connector
from datetime import datetime

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
'''
mycursor.execute("CREATE TABLE history (id int AUTO_INCREMENT PRIMARY KEY, time DATETIME, enterCount smallint unsigned, exitCount smallint unsigned)")

mycursor.execute("SHOW TABLES")

for x in mycursor:
  print(x)
'''
peopleEntered = 5
peopleExited = 3
sql = "INSERT INTO history (time, enterCount, exitCount) VALUES (%s, %s, %s)"
time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
mysqlVal = (time, peopleEntered, peopleExited)

mycursor.execute(sql, mysqlVal)
#print(dt)
mydb.commit()

mycursor.execute("""
  SELECT time, enterCount, exitCount FROM history 
  WHERE time between '2012-03-11 00:00:00' and '2022-05-11 23:59:00'
  order by time desc
  """)

myresult = mycursor.fetchall()

print(myresult)