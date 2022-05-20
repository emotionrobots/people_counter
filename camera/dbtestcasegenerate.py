import mysql.connector
from datetime import datetime
import random

mydb = mysql.connector.connect(
  host="localhost",
  user="emotioneering",
  password="password",
  database="history"
)

sql = "INSERT INTO history (time, enterCount, exitCount) VALUES (%s, %s, %s)"

mycursor = mydb.cursor()

for x in range(198):

  dt = datetime.utcnow().isoformat()

  peopleEntered = random.randrange(11, 40)
  peopleExited = random.randrange(0,15)

  mysqlVal = (dt, peopleEntered, peopleExited)
  mycursor.execute(sql, mysqlVal)
  print(x, mysqlVal)

  mydb.commit()
