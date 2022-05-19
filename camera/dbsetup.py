import mysql.connector
from datetime import datetime

#===========================================================================
# mysql connector and cursor setup
#===========================================================================
mydb = mysql.connector.connect(
  host="localhost",
  user="emotioneering",
  password="password",
  database="history"
)

mycursor = mydb.cursor()

#===========================================================================
# creates DB on first run
#===========================================================================
'''
#mycursor.execute("CREATE DATABASE history")

#mycursor.execute("SHOW DATABASES")

#for x in mycursor:
  #print(x)
'''

#===========================================================================
# creates table
#===========================================================================
'''
mycursor.execute("""
CREATE TABLE history 
(id int AUTO_INCREMENT PRIMARY KEY, 
time DATETIME, 
enterCount smallint unsigned, 
exitCount smallint unsigned)""")


mycursor.execute("SHOW TABLES")

for x in mycursor:
  print(x)
'''

#===========================================================================
# creates autodelete event that reruns everyday delete rows older than 60 days
#===========================================================================
mycursor.execute("DROP EVENT AutoDeleteOldNotifications")

'''
mycursor.execute("""
CREATE EVENT AutoDeleteOldNotifications
ON SCHEDULE AT CURRENT_TIMESTAMP + INTERVAL 1 DAY 
ON COMPLETION PRESERVE
DO 
DELETE LOW_PRIORITY FROM history WHERE time < DATE_SUB(NOW(), INTERVAL 60 DAY)""")
'''

#===========================================================================
# manually adds entries for testing
#===========================================================================
peopleEntered = 2
peopleExited = 38
sql = "INSERT INTO history (time, enterCount, exitCount) VALUES (%s, %s, %s)"
time = (datetime.now()).strftime("%Y-%m-%d %H:%M:%S")
print(time)
mysqlVal = (time, peopleEntered, peopleExited)

mycursor.execute(sql, mysqlVal)
#print(dt)
mydb.commit()

#===========================================================================
# outputs all the rows from table
#===========================================================================
mycursor.execute("""
  SELECT time, enterCount, exitCount FROM history 
  WHERE time between '2012-03-11 00:00:00' and '2022-05-11 23:59:00'
  order by time desc
  """)

myresult = mycursor.fetchall()

for x in myresult:
  print(x[0].strftime("%Y-%m-%d %H:%M:%S"), x[1], x[2])