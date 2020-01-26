#!/usr/bin/env python3

import sys

import tornado.ioloop
import tornado.websocket

class SocketHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        pass

    def on_message(self, message):
        pass

    def on_close(self):
        pass

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: {} <port>".format(sys.argv[0]))
        raise SystemExit(1)
    port = int(sys.argv[1])
    application = tornado.web.Application([
        (r"/", SocketHandler),
    ])
    application.listen(port)
    tornado.ioloop.IOLoop.current().start()
