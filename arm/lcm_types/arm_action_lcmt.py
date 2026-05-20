import struct


class arm_action_lcmt:
    _HASH = -3288094115613904388

    def __init__(self):
        self.act = [0.0] * 14

    def encode(self):
        values = list(self.act)
        if len(values) != 14:
            raise ValueError("arm_action_lcmt.act must contain 14 values")
        return struct.pack(">q14d", self._HASH, *values)

    @staticmethod
    def decode(data):
        unpacked = struct.unpack(">q14d", data[:120])
        if unpacked[0] != arm_action_lcmt._HASH:
            raise ValueError("bad arm_action_lcmt hash")
        msg = arm_action_lcmt()
        msg.act = list(unpacked[1:])
        return msg
