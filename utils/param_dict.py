from collections import OrderedDict
from io import StringIO
from typing import TextIO
from warnings import warn

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

    def __getitem__(self, idx: str | int) -> float:
        if isinstance(idx, str):
            if len(idx) > 16:
                raise KeyError('param name too long')
            return self._data[idx]
        elif type(idx) == int:
            if idx >= len(self._data):
                raise IndexError('param index out of range')
            # HACK: is there a nicer way to get items from OrderedDict by order?
            name = list(self._data.keys())[idx]
            return self._data[name]
        else:
            raise TypeError('Required parameter name or index')

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
        elif type(idx) == int:
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

    def dump(self, fp: TextIO):
        """Serialize values into a .param text file"""
        kv = [(k, self[k]) for k in self.keys()]
        kv.sort()  # .param files are sorted by parameter name
        for k, v in kv:
            fp.write(f'{k},{v}\n')

    def dumps(self) -> str:
        """Serialize values into string like .param text file"""
        with StringIO() as buffer:
            self.dump(buffer)
            return buffer.getvalue()

    def load(self, fp: TextIO):
        """Read values from .param text file

        All parameter names not defined yet will be appended at the end in the same order as they appear in file"""
        kv: list[tuple[str, float]] = []
        for line_num, line in enumerate(fp.readlines()):
            line = line.strip()
            if line == '':
                continue

            if ',' not in line:
                warn(f'no comma in line {line_num + 1}: {line}')
                continue

            parts = line.split(',')
            if len(parts) != 2:
                warn(f'too many parts in line {line_num + 1}: {line}')
                continue

            name = parts[0]
            value: float
            try:
                value = float(parts[1])
            except ValueError:
                warn(f'could not parse value in line {line_num + 1}: {line}')
                continue

            kv.append((name, value))

        # Update defined values
        for name, value in kv:
            if name not in self:
                continue
            self[name] = value

        # Add new values
        for name, value in kv:
            if name in self:
                continue
            self[name] = value

    def loads(self, s: str):
        """Read values from .param file text, see also load()"""
        with StringIO(s) as buffer:
            self.load(buffer)
