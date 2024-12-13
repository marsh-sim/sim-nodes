from collections import OrderedDict

import mavlink_all as mavlink


class ParamDict:
    """
    Object for storing parameters, intended to be used from code like a regular dictonary.
    The order of assigning new parameters will be reflected in `param_index` field of MAVLink messages.

    For detailed documentation of the protocol, see
    https://mavlink.io/en/services/parameter.html
    """

    def __init__(self):
        self._data: OrderedDict[str, float] = OrderedDict()
        """Dictionary storing the actual values, do not modify directly"""
        self.readonly_names: set[str] = set()
        """Parameter names that can't be modified by MAVLink messages"""

    def __len__(self):
        return self._data.__len__()

    def __getitem__(self, idx: str | int):
        if isinstance(idx, str):
            if len(idx) > 16:
                raise KeyError('param name too long')
            return self._data[idx]
        elif isinstance(idx, int):
            if idx >= len(self._data):
                raise IndexError('param index out of range')
            # HACK: is there a nicer way to get items from OrderedDict by order?
            name = list(self._data.keys())[idx]
            return self._data[name]

    def __setitem__(self, name: str, value: float):
        if len(name) > 16:
            raise KeyError('param name too long')
        self._data[name] = float(value)

    def __contains__(self, name: str):
        return self._data.__contains__(name)

    def keys(self):
        return self._data.keys()

    @staticmethod
    def should_handle_message(message: mavlink.MAVLink_message):
        return message.get_type() in ['PARAM_REQUEST_READ', 'PARAM_REQUEST_LIST', 'PARAM_SET']

    def handle_message(self, mav: mavlink.MAVLink, message: mavlink.MAVLink_message) -> None | tuple[str, float]:
        """Handle all applicable messages, returns a parameter change if it occured"""
        assert self.should_handle_message(message), \
            'Check message type before calling'

        # discard messages not relevant to us
        if message.target_system != mav.srcSystem or message.target_component != mav.srcComponent:
            return None

        if message.get_type() == 'PARAM_REQUEST_READ':
            m: mavlink.MAVLink_param_request_read_message = message
            if m.param_index == -1:
                self.send_param(mav, m.param_id)
            elif 0 <= m.param_index < len(self):
                self.send_param(mav, m.param_index)

        elif message.get_type() == 'PARAM_REQUEST_LIST':
            for i in range(len(self)):
                self.send_param(mav, i)

        elif message.get_type() == 'PARAM_SET':
            m: mavlink.MAVLink_param_set_message = message
            changed = False
            # check that parameter is defined and sent as float
            if m.param_id in self and m.param_type == mavlink.MAV_PARAM_TYPE_REAL32:
                # don't write to read-only parameters but report the value
                if m.param_id not in self.readonly_names and self[m.param_id] != m.param_value:
                    self[m.param_id] = m.param_value
                    changed = True

            # reply with current value to all requests for an existing param, to confirm reception
            if m.param_id in self:
                self.send_param(mav, m.param_id)

            if changed:
                return m.param_id, self[m.param_id]

        return None

    def send_param(self, mav: mavlink.MAVLink, idx: str | int):
        param_index: int
        name: str
        if isinstance(idx, str):
            if len(idx) > 16:
                raise KeyError('param name too long')
            name = idx
            param_index = list(self._data.keys()).index(name)
        elif isinstance(idx, int):
            if idx < 0 or idx >= len(self):
                raise IndexError('param index out of range')
            param_index = idx
            name = list(self._data.keys())[param_index]
        else:
            return

        param_id = bytearray(16)
        name_bytes = name.encode('utf8')
        param_id[:len(name_bytes)] = name_bytes

        mav.param_value_send(bytes(param_id), self[name],
                             mavlink.MAV_PARAM_TYPE_REAL32,
                             len(self), param_index)
