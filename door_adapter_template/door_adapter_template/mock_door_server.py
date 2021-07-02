#! /usr/bin/env python3

import flask
from flask import request, jsonify

app = flask.Flask(__name__)
app.config["DEBUG"] = True

@app.route('/door/status', methods=['POST'])
def device_status():
    data = {"statusCode": 200,"isBase64Encoded": False,"headers": {"Content-Type": "application/json","Access-Control-Allow-Origin": "*"  }, "body": {"doorState": "closed"}}
    return jsonify(data)

@app.route('/door/remoteopen', methods=['POST'])
def door_open_command():
    data = {"statusCode": 200,"isBase64Encoded": False,"headers": {"Content-Type": "application/json","Access-Control-Allow-Origin": "*"  }, "body": {"result":"open command sent"}}
    return jsonify(data)


app.run(port=8888)