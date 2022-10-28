import json

class Util:

    def __init__(self, verbose=False, log=False) -> None:
        self.verbose = verbose
        self.save_log = log
        self.limitInterval = lambda u, a, b: b if u > b else a if u < a else u
        self.wsSend = lambda ws, msg: ws.send(json.dumps(msg))
        self.wsPub = lambda ws, topic, data: self.wsSend(ws, {
            'op': 'publish',
            'topic': topic,
            'msg':{
                'data': data,
            },
        })
        self.orb_tracking_status = {
            '-1': 'SYSTEM_NOT_READY',
            '0': 'NO_IMAGE_YET',
            '1': 'NOT_INITIALIZED',
            '2': 'OK',
            '3': 'LOST',
        }

    def log(self, filepath, str):
        if(self.save_log):
            with open(filepath, 'a') as f:
                f.write(str)
            f.close()

    def print(self, str):
        if(self.verbose):
            print(str)