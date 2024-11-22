"""Example web app"""

import tornado.web
import sqlite3
import bcrypt
import random

conn = sqlite3.connect('tri.db')
cur = conn.cursor()

# Database connection and initialization
try:
    cur.execute('''CREATE TABLE users (name text, password text, \
                answer int)''')
    username = "name"
    pwd = "password".encode("utf-8")
    answer = 0
    pwd_hash = bcrypt.hashpw(pwd, bcrypt.gensalt())
    cur.execute("INSERT INTO users VALUES (?, ?, ?)", [
        username,
        pwd_hash,
        answer
    ])
    conn.commit()
except sqlite3.OperationalError:
    pass


class MainHandler(tornado.web.RequestHandler):
    """ Main request handler """

    def get(self):
        """ GET request handler """

        number_one = random.randint(0, 999)
        number_two = random.randint(0, 999)
        response_string = ("<html><body>"
                           "Number one: " + str(number_one) + "<br>"
                           "Number two: " + str(number_two) + "<br>"
                           "<form action='/' method='POST'>"
                           "Name: <input type='text' name='name'><br>"
                           "Password: <input type='text' name='password'><br>"
                           "Answer: <input type='number' name='answer'><br>"
                           "<input type='submit' value='Submit'>"
                           "</form></body></html>")
        self.write(response_string)

    def post(self):
        """ POST request handler """

        username = self.get_argument("name")
        password = self.get_argument("password")
        answer = self.get_argument("answer")
        cur.execute("SELECT password FROM users WHERE name = ?", [username])
        hashed = cur.fetchone()
        if hashed is not None:
            encpw = password.encode("utf-8")
            result = bcrypt.checkpw(encpw, hashed[0])
            if result:
                cur.execute("update users set answer = ? where name = ?",
                            (answer, username))
                conn.commit()
                cur.execute("SELECT answer FROM users WHERE name = ?",
                            [username])
                answer = cur.fetchone()
                print("answer saved: ", answer)
                self.write("Success")
            else:
                self.write("Auth failure")
        else:
            self.write("Name not found")


if __name__ == "__main__":
    """ Main """

    app = tornado.web.Application([
        (r"/", MainHandler),
    ])
    server = tornado.httpserver.HTTPServer(app)
    server.listen(8888)
    tornado.ioloop.IOLoop.instance().start()
