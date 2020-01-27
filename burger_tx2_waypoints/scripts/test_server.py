#!/usr/bin/env python3

import json
import os.path
import queue
import sys

import tornado.ioloop
import tornado.websocket


JSON_ERROR = 127
WRONG_TYPE = 128
MISSING_MEMBER = 129

ERROR_FORMATS = {
    JSON_ERROR: "Json error in incoming message: {!r}",
    WRONG_TYPE: "Wrong incoming message type: {!r}",
    MISSING_MEMBER: "Missing required message member: {!r}"
}


outbox = queue.SimpleQueue()


class Robot:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.location_time_stamp = 0
        self.latitude = 0.0
        self.longitude = 0.0
        self.robot_status = "stopped"
        self.waypoints_name = None

    def set_location(self, stamp, lat, lon):
        self.location_time_stamp = stamp
        self.latitude = lat
        self.longitude = lon

    def set_status(self, status, wayname):
        self.robot_status = status
        self.waypoints_name = wayname


class SocketHandler (tornado.websocket.WebSocketHandler):
    def open(self):
        self.robots = {}

    def send_queued(self):
        try:
            while True:
                mobj = outbox.get_nowait()
                print("Recieved message:")
                print(json.dumps(mobj, indent=2, sort_keys=True))
                message = json.dumps(mobj)
                self.write_message(message)
        except queue.Empty:
            pass

    def on_message(self, message):
        try:
            mobj = json.loads(message)
        except json.JSONDecodeError as e:
            self.write_error(JSON_ERROR, e)
            return
        msg_type = mobj.get("msg_type")
        print("Recieved message:")
        print(json.dumps(mobj, indent=2, sort_keys=True))
        try:
            if msg_type == "report_location":
                rid = mobj["robot_id"]
                stamp = mobj["time_stamp"]
                lat = mobj["location"]["latitudes"]
                lon = mobj["location"]["longitudes"]
                if rid not in self.robots:
                    self.robots[rid] = Robot(rid)
                self.robots[rid].set_location(stamp, lat, lon)
                self.send_queued()
            elif msg_type == "robot_status_answer":
                rid = mobj["robot_id"]
                status = mobj["robot_status"]
                wayname = mobj.get("waypoints_name")
                if rid not in self.robots:
                    self.robots[rid] = Robot(rid)
                self.robots[rid].set_status(status, wayname)
            else:
                self.write_error(WRONG_TYPE, msg_type)
        except KeyError as e:
            self.write_error(MISSING_MEMBER, e)

    def write_error(self, code, *args, **kwargs):
        error = {
            "msg_type": "error",
            "error_code": code,
            "error_info": ERROR_FORMATS[code].format(*args, **kwargs)
        }
        print("ERROR:")
        print(json.dumps(error, indent=2, sort_keys=True))
        message = json.dumps(error)
        return self.write_message(message)

    def on_close(self):
        print("Closing. Final robots' state:")
        print(json.dumps(mobj, indent=2, sort_keys=True))
        self.robots = {}


class RemoteControlHandler (tornado.web.RequestHandler):
    def prepare(self):
        self.set_status(301)
        self.set_header("Location", "/rc/index.html")

    def post(self):
        msg_type = self.get_argument("msg_type", None)
        message = { "msg_type": msg_type }
        if msg_type == "waypoints_travel":
            message["waypoints_name"] = self.get_argument(
                "waypoints_name", "")
            message["waypoints"] = []
            latitudes = self.get_arguments("latitudes")
            longitudes = self.get_arguments("longitudes")
            for lat, lon in zip(latitudes, longitudes):
                message["waypoints"].append({
                    "latitudes": lat,
                    "longitudes": lon
                })
        elif msg_type == "robot_status_change":
            message["change_to_status"] = self.get_argument(
                "change_to_status", "Paused").lower()
        elif msg_type == "robot_status_query":
            pass
        else:
            return
        print("Queueing {} message.".format(msg_type))
        outbox.put(message)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: {} <port>".format(sys.argv[0]))
        raise SystemExit(1)
    port = int(sys.argv[1])
    rcdir = os.path.abspath(os.path.join(os.path.dirname(__file__), "rc"))
    application = tornado.web.Application([
        ("/", SocketHandler),
        ("/rc", RemoteControlHandler),
        ("/rc/(.*)", tornado.web.StaticFileHandler, {"path": rcdir})
    ])
    application.listen(port)
    tornado.ioloop.IOLoop.current().start()
