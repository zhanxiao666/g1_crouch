import struct


class body_control_data_lcmt:
    _HASH = -7756648662117236310

    def __init__(self):
        self.q = [0.0] * 29
        self.qd = [0.0] * 29
        self.root_quat_w = [1.0, 0.0, 0.0, 0.0]
        self.timestamp_us = 0

    def encode(self):
        q = list(self.q)
        qd = list(self.qd)
        root_quat_w = list(self.root_quat_w)
        if len(q) != 29 or len(qd) != 29 or len(root_quat_w) != 4:
            raise ValueError("body_control_data_lcmt.q, qd, and root_quat_w have invalid sizes")
        return struct.pack(">q29f29f4fq", self._HASH, *q, *qd, *root_quat_w, int(self.timestamp_us))

    @staticmethod
    def decode(data):
        unpacked = struct.unpack(">q29f29f4fq", data[:264])
        if unpacked[0] != body_control_data_lcmt._HASH:
            raise ValueError("bad body_control_data_lcmt hash")
        msg = body_control_data_lcmt()
        msg.q = list(unpacked[1:30])
        msg.qd = list(unpacked[30:59])
        msg.root_quat_w = list(unpacked[59:63])
        msg.timestamp_us = unpacked[63]
        return msg
