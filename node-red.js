[
    {
        "id": "6147016.8ff64",
        "type": "ttn uplink",
        "z": "613373c4.05758c",
        "name": "Change me",
        "app": "fc69d206.487df",
        "dev_id": "",
        "field": "",
        "x": 80,
        "y": 1180,
        "wires": [
            [
                "84fe7d0b.6c384",
                "24886461.8e6bfc"
            ]
        ]
    },
    {
        "id": "84fe7d0b.6c384",
        "type": "function",
        "z": "613373c4.05758c",
        "name": "",
        "func": "var maxValue = -200;\nvar indexOfMaxValue = -1;\n    for(var i = 0; i < msg.metadata.gateways.length; i++){\n        if(msg.metadata.gateways[i].rssi > maxValue){\n            indexOfMaxValue = i;\n        }\n    }\n\n//rssi\nvar msg1 = { payload: msg.payload.length };\nmsg1.payload = msg.metadata.gateways[indexOfMaxValue].rssi;\n\n//snr\nvar msg2 = { payload: msg.payload.length };\nmsg2.payload = msg.metadata.gateways[indexOfMaxValue].snr;\n\nif (msg.payload.batt === undefined ){\nvar msg3 = {};\nmsg3.payload = [{\"rssi\": msg1.payload, \"snr\": msg2.payload}];\n}\nelse {\nvar msg4 = { payload: msg.payload.length };\nmsg4.payload = msg.payload.batt;    \n    var msg3 = {};\nmsg3.payload = [{\"rssi\": msg1.payload, \"snr\": msg2.payload, \"batt\":msg4.payload}];\n}\n\nreturn [msg3];\n\n\n",
        "outputs": 1,
        "noerr": 0,
        "x": 230,
        "y": 1180,
        "wires": [
            [
                "8bb4a83d.645908"
            ]
        ]
    },
    {
        "id": "8bb4a83d.645908",
        "type": "influxdb out",
        "z": "613373c4.05758c",
        "influxdb": "6b5d7087.5f161",
        "name": "",
        "measurement": "Change me",
        "precision": "",
        "retentionPolicy": "",
        "x": 510,
        "y": 1180,
        "wires": []
    },
    {
        "id": "af67a001.6d523",
        "type": "influxdb out",
        "z": "613373c4.05758c",
        "influxdb": "24f90c8.e6ee3f4",
        "name": "",
        "measurement": "Change me",
        "precision": "",
        "retentionPolicy": "",
        "x": 770,
        "y": 1220,
        "wires": []
    },
    {
        "id": "f19fa4ca.7be098",
        "type": "switch",
        "z": "613373c4.05758c",
        "name": "",
        "property": "payload.lat",
        "propertyType": "msg",
        "rules": [
            {
                "t": "eq",
                "v": "-90",
                "vt": "num"
            },
            {
                "t": "else"
            }
        ],
        "checkall": "true",
        "repair": false,
        "outputs": 2,
        "x": 450,
        "y": 1220,
        "wires": [
            [],
            [
                "c5963905.6ed648"
            ]
        ]
    },
    {
        "id": "c5963905.6ed648",
        "type": "geohash",
        "z": "613373c4.05758c",
        "name": "",
        "x": 580,
        "y": 1220,
        "wires": [
            [
                "af67a001.6d523"
            ]
        ]
    },
    {
        "id": "24886461.8e6bfc",
        "type": "function",
        "z": "613373c4.05758c",
        "name": "Decrypt Payload",
        "func": "//lat\nif (msg.payload.lat === undefined){}\nelse{\nvar msg1 = { payload: msg.payload.length };\nmsg1.payload = msg.payload.lat\n\n//lon\nvar msg2 = { payload: msg.payload.length };\nmsg2.payload = msg.payload.lon;\n\n//Output\nvar msg6 = {};\nmsg6.payload = {\"lat\": msg1.payload, \"lon\": msg2.payload};\nvar msg7 = {};\nmsg7.payload = \"Bike Alarm - Movement detected\";\n\nreturn [msg6,msg7];\n}\n\n",
        "outputs": 2,
        "noerr": 0,
        "x": 260,
        "y": 1240,
        "wires": [
            [
                "f19fa4ca.7be098"
            ],
            [
                "3c1fa57c.d78b4a"
            ]
        ]
    },
    {
        "id": "eb8bc5a9.5a94a8",
        "type": "ttn downlink",
        "z": "613373c4.05758c",
        "name": "Alarm Off",
        "app": "fc69d206.487df",
        "dev_id": "Change me",
        "port": "1",
        "confirmed": false,
        "schedule": "replace",
        "x": 460,
        "y": 1300,
        "wires": []
    },
    {
        "id": "79e78fc8.52b45",
        "type": "inject",
        "z": "613373c4.05758c",
        "name": "",
        "topic": "",
        "payload": "",
        "payloadType": "date",
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "x": 100,
        "y": 1300,
        "wires": [
            [
                "ef9f54ca.aad138"
            ]
        ]
    },
    {
        "id": "ef9f54ca.aad138",
        "type": "function",
        "z": "613373c4.05758c",
        "name": "Alarm off",
        "func": "return {\n  dev_id: msg.dev_id,\n  port: msg.port,\n  payload: \"00\"\n}",
        "outputs": 1,
        "noerr": 0,
        "x": 260,
        "y": 1300,
        "wires": [
            [
                "eb8bc5a9.5a94a8"
            ]
        ]
    },
    {
        "id": "c53fb9a4.e74d38",
        "type": "pushover",
        "z": "613373c4.05758c",
        "name": "Alarm",
        "device": "",
        "title": "Alarm",
        "priority": "2",
        "sound": "siren",
        "url": "",
        "url_title": "",
        "html": false,
        "x": 710,
        "y": 1260,
        "wires": []
    },
    {
        "id": "3c1fa57c.d78b4a",
        "type": "delay",
        "z": "613373c4.05758c",
        "name": "",
        "pauseType": "rate",
        "timeout": "5",
        "timeoutUnits": "seconds",
        "rate": "1",
        "nbRateUnits": "15",
        "rateUnits": "minute",
        "randomFirst": "1",
        "randomLast": "5",
        "randomUnits": "seconds",
        "drop": true,
        "x": 480,
        "y": 1260,
        "wires": [
            [
                "c53fb9a4.e74d38"
            ]
        ]
    },
    {
        "id": "fc69d206.487df",
        "type": "ttn app",
        "z": "",
        "appId": "Change me",
        "accessKey": "Change me",
        "discovery": "discovery.thethingsnetwork.org:1900"
    },
    {
        "id": "6b5d7087.5f161",
        "type": "influxdb",
        "z": "",
        "hostname": "127.0.0.1",
        "port": "8086",
        "protocol": "http",
        "database": "nodes",
        "name": "",
        "usetls": false,
        "tls": ""
    },
    {
        "id": "24f90c8.e6ee3f4",
        "type": "influxdb",
        "z": "",
        "hostname": "127.0.0.1",
        "port": "8086",
        "protocol": "http",
        "database": "gps",
        "name": "",
        "usetls": false,
        "tls": ""
    }
]
