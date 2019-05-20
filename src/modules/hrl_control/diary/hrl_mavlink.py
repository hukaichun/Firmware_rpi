'''
MAVLink protocol implementation (auto-generated by mavgen.py)

Generated from: control_info.xml

Note: this file has been auto-generated. DO NOT EDIT
'''
from __future__ import print_function
from builtins import range
from builtins import object
import struct, array, time, json, os, sys, platform

from ...generator.mavcrc import x25crc
import hashlib

WIRE_PROTOCOL_VERSION = '2.0'
DIALECT = 'hrl_mavlink'

PROTOCOL_MARKER_V1 = 0xFE
PROTOCOL_MARKER_V2 = 0xFD
HEADER_LEN_V1 = 6
HEADER_LEN_V2 = 10

MAVLINK_SIGNATURE_BLOCK_LEN = 13

MAVLINK_IFLAG_SIGNED = 0x01

native_supported = platform.system() != 'Windows' # Not yet supported on other dialects
native_force = 'MAVNATIVE_FORCE' in os.environ # Will force use of native code regardless of what client app wants
native_testing = 'MAVNATIVE_TESTING' in os.environ # Will force both native and legacy code to be used and their results compared

if native_supported and float(WIRE_PROTOCOL_VERSION) <= 1:
    try:
        import mavnative
    except ImportError:
        print('ERROR LOADING MAVNATIVE - falling back to python implementation')
        native_supported = False
else:
    # mavnative isn't supported for MAVLink2 yet
    native_supported = False

# some base types from mavlink_types.h
MAVLINK_TYPE_CHAR     = 0
MAVLINK_TYPE_UINT8_T  = 1
MAVLINK_TYPE_INT8_T   = 2
MAVLINK_TYPE_UINT16_T = 3
MAVLINK_TYPE_INT16_T  = 4
MAVLINK_TYPE_UINT32_T = 5
MAVLINK_TYPE_INT32_T  = 6
MAVLINK_TYPE_UINT64_T = 7
MAVLINK_TYPE_INT64_T  = 8
MAVLINK_TYPE_FLOAT    = 9
MAVLINK_TYPE_DOUBLE   = 10


class MAVLink_header(object):
    '''MAVLink message header'''
    def __init__(self, msgId, incompat_flags=0, compat_flags=0, mlen=0, seq=0, srcSystem=0, srcComponent=0):
        self.mlen = mlen
        self.seq = seq
        self.srcSystem = srcSystem
        self.srcComponent = srcComponent
        self.msgId = msgId
        self.incompat_flags = incompat_flags
        self.compat_flags = compat_flags

    def pack(self, force_mavlink1=False):
        if WIRE_PROTOCOL_VERSION == '2.0' and not force_mavlink1:
            return struct.pack('<BBBBBBBHB', 253, self.mlen,
                               self.incompat_flags, self.compat_flags,
                               self.seq, self.srcSystem, self.srcComponent,
                               self.msgId&0xFFFF, self.msgId>>16)
        return struct.pack('<BBBBBB', PROTOCOL_MARKER_V1, self.mlen, self.seq,
                           self.srcSystem, self.srcComponent, self.msgId)

class MAVLink_message(object):
    '''base MAVLink message class'''
    def __init__(self, msgId, name):
        self._header     = MAVLink_header(msgId)
        self._payload    = None
        self._msgbuf     = None
        self._crc        = None
        self._fieldnames = []
        self._type       = name
        self._signed     = False
        self._link_id    = None

    def format_attr(self, field):
        '''override field getter'''
        raw_attr = getattr(self,field)
        if isinstance(raw_attr, bytes):
            raw_attr = raw_attr.decode("utf-8").rstrip("\00")
        return raw_attr

    def get_msgbuf(self):
        if isinstance(self._msgbuf, bytearray):
            return self._msgbuf
        return bytearray(self._msgbuf)

    def get_header(self):
        return self._header

    def get_payload(self):
        return self._payload

    def get_crc(self):
        return self._crc

    def get_fieldnames(self):
        return self._fieldnames

    def get_type(self):
        return self._type

    def get_msgId(self):
        return self._header.msgId

    def get_srcSystem(self):
        return self._header.srcSystem

    def get_srcComponent(self):
        return self._header.srcComponent

    def get_seq(self):
        return self._header.seq

    def get_signed(self):
        return self._signed

    def get_link_id(self):
        return self._link_id

    def __str__(self):
        ret = '%s {' % self._type
        for a in self._fieldnames:
            v = self.format_attr(a)
            ret += '%s : %s, ' % (a, v)
        ret = ret[0:-2] + '}'
        return ret

    def __ne__(self, other):
        return not self.__eq__(other)

    def __eq__(self, other):
        if other is None:
            return False

        if self.get_type() != other.get_type():
            return False

        # We do not compare CRC because native code doesn't provide it
        #if self.get_crc() != other.get_crc():
        #    return False

        if self.get_seq() != other.get_seq():
            return False

        if self.get_srcSystem() != other.get_srcSystem():
            return False

        if self.get_srcComponent() != other.get_srcComponent():
            return False

        for a in self._fieldnames:
            if self.format_attr(a) != other.format_attr(a):
                return False

        return True

    def to_dict(self):
        d = dict({})
        d['mavpackettype'] = self._type
        for a in self._fieldnames:
          d[a] = self.format_attr(a)
        return d

    def to_json(self):
        return json.dumps(self.to_dict())

    def sign_packet(self, mav):
        h = hashlib.new('sha256')
        self._msgbuf += struct.pack('<BQ', mav.signing.link_id, mav.signing.timestamp)[:7]
        h.update(mav.signing.secret_key)
        h.update(self._msgbuf)
        sig = h.digest()[:6]
        self._msgbuf += sig
        mav.signing.timestamp += 1

    def pack(self, mav, crc_extra, payload, force_mavlink1=False):
        plen = len(payload)
        if WIRE_PROTOCOL_VERSION != '1.0' and not force_mavlink1:
            # in MAVLink2 we can strip trailing zeros off payloads. This allows for simple
            # variable length arrays and smaller packets
            while plen > 1 and payload[plen-1] == chr(0):
                plen -= 1
        self._payload = payload[:plen]
        incompat_flags = 0
        if mav.signing.sign_outgoing:
            incompat_flags |= MAVLINK_IFLAG_SIGNED
        self._header  = MAVLink_header(self._header.msgId,
                                       incompat_flags=incompat_flags, compat_flags=0,
                                       mlen=len(self._payload), seq=mav.seq,
                                       srcSystem=mav.srcSystem, srcComponent=mav.srcComponent)
        self._msgbuf = self._header.pack(force_mavlink1=force_mavlink1) + self._payload
        crc = x25crc(self._msgbuf[1:])
        if True: # using CRC extra
            crc.accumulate_str(struct.pack('B', crc_extra))
        self._crc = crc.crc
        self._msgbuf += struct.pack('<H', self._crc)
        if mav.signing.sign_outgoing and not force_mavlink1:
            self.sign_packet(mav)
        return self._msgbuf


# enums

class EnumEntry(object):
    def __init__(self, name, description):
        self.name = name
        self.description = description
        self.param = {}

enums = {}

# message IDs
MAVLINK_MSG_ID_BAD_DATA = -1
MAVLINK_MSG_ID_CONTROL_INFO = 0
MAVLINK_MSG_ID_NN_LIB_INFO = 1

class MAVLink_control_info_message(MAVLink_message):
        '''
        state action pair
        '''
        id = MAVLINK_MSG_ID_CONTROL_INFO
        name = 'CONTROL_INFO'
        fieldnames = ['timestamp', 'quaternion', 'position', 'angular_velocity', 'velocity', 'control_h', 'control_l']
        ordered_fieldnames = ['timestamp', 'quaternion', 'position', 'angular_velocity', 'velocity', 'control_h', 'control_l']
        fieldtypes = ['uint64_t', 'float', 'float', 'float', 'float', 'float', 'float']
        format = '<Q4f3f3f3f3f4f'
        native_format = bytearray('<Qffffff', 'ascii')
        orders = [0, 1, 2, 3, 4, 5, 6]
        lengths = [1, 4, 3, 3, 3, 3, 4]
        array_lengths = [0, 4, 3, 3, 3, 3, 4]
        crc_extra = 196
        unpacker = struct.Struct('<Q4f3f3f3f3f4f')

        def __init__(self, timestamp, quaternion, position, angular_velocity, velocity, control_h, control_l):
                MAVLink_message.__init__(self, MAVLink_control_info_message.id, MAVLink_control_info_message.name)
                self._fieldnames = MAVLink_control_info_message.fieldnames
                self.timestamp = timestamp
                self.quaternion = quaternion
                self.position = position
                self.angular_velocity = angular_velocity
                self.velocity = velocity
                self.control_h = control_h
                self.control_l = control_l

        def pack(self, mav, force_mavlink1=False):
                return MAVLink_message.pack(self, mav, 196, struct.pack('<Q4f3f3f3f3f4f', self.timestamp, self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3], self.position[0], self.position[1], self.position[2], self.angular_velocity[0], self.angular_velocity[1], self.angular_velocity[2], self.velocity[0], self.velocity[1], self.velocity[2], self.control_h[0], self.control_h[1], self.control_h[2], self.control_l[0], self.control_l[1], self.control_l[2], self.control_l[3]), force_mavlink1=force_mavlink1)

class MAVLink_nn_lib_info_message(MAVLink_message):
        '''
        update nn shared obj
        '''
        id = MAVLINK_MSG_ID_NN_LIB_INFO
        name = 'NN_LIB_INFO'
        fieldnames = [' D_timestamp', 'dir']
        ordered_fieldnames = [' D_timestamp', 'dir']
        fieldtypes = ['uint64_t', 'char']
        format = '<Q128s'
        native_format = bytearray('<Qc', 'ascii')
        orders = [0, 1]
        lengths = [1, 1]
        array_lengths = [0, 128]
        crc_extra = 82
        unpacker = struct.Struct('<Q128s')

        def __init__(self,  D_timestamp, dir):
                MAVLink_message.__init__(self, MAVLink_nn_lib_info_message.id, MAVLink_nn_lib_info_message.name)
                self._fieldnames = MAVLink_nn_lib_info_message.fieldnames
                self. D_timestamp =  D_timestamp
                self.dir = dir

        def pack(self, mav, force_mavlink1=False):
                return MAVLink_message.pack(self, mav, 82, struct.pack('<Q128s', self. D_timestamp, self.dir), force_mavlink1=force_mavlink1)


mavlink_map = {
        MAVLINK_MSG_ID_CONTROL_INFO : MAVLink_control_info_message,
        MAVLINK_MSG_ID_NN_LIB_INFO : MAVLink_nn_lib_info_message,
}

class MAVError(Exception):
        '''MAVLink error class'''
        def __init__(self, msg):
            Exception.__init__(self, msg)
            self.message = msg

class MAVString(str):
        '''NUL terminated string'''
        def __init__(self, s):
                str.__init__(self)
        def __str__(self):
            i = self.find(chr(0))
            if i == -1:
                return self[:]
            return self[0:i]

class MAVLink_bad_data(MAVLink_message):
        '''
        a piece of bad data in a mavlink stream
        '''
        def __init__(self, data, reason):
                MAVLink_message.__init__(self, MAVLINK_MSG_ID_BAD_DATA, 'BAD_DATA')
                self._fieldnames = ['data', 'reason']
                self.data = data
                self.reason = reason
                self._msgbuf = data

        def __str__(self):
            '''Override the __str__ function from MAVLink_messages because non-printable characters are common in to be the reason for this message to exist.'''
            return '%s {%s, data:%s}' % (self._type, self.reason, [('%x' % ord(i) if isinstance(i, str) else '%x' % i) for i in self.data])

class MAVLinkSigning(object):
    '''MAVLink signing state class'''
    def __init__(self):
        self.secret_key = None
        self.timestamp = 0
        self.link_id = 0
        self.sign_outgoing = False
        self.allow_unsigned_callback = None
        self.stream_timestamps = {}
        self.sig_count = 0
        self.badsig_count = 0
        self.goodsig_count = 0
        self.unsigned_count = 0
        self.reject_count = 0

class MAVLink(object):
        '''MAVLink protocol handling class'''
        def __init__(self, file, srcSystem=0, srcComponent=0, use_native=False):
                self.seq = 0
                self.file = file
                self.srcSystem = srcSystem
                self.srcComponent = srcComponent
                self.callback = None
                self.callback_args = None
                self.callback_kwargs = None
                self.send_callback = None
                self.send_callback_args = None
                self.send_callback_kwargs = None
                self.buf = bytearray()
                self.buf_index = 0
                self.expected_length = HEADER_LEN_V1+2
                self.have_prefix_error = False
                self.robust_parsing = False
                self.protocol_marker = 253
                self.little_endian = True
                self.crc_extra = True
                self.sort_fields = True
                self.total_packets_sent = 0
                self.total_bytes_sent = 0
                self.total_packets_received = 0
                self.total_bytes_received = 0
                self.total_receive_errors = 0
                self.startup_time = time.time()
                self.signing = MAVLinkSigning()
                if native_supported and (use_native or native_testing or native_force):
                    print("NOTE: mavnative is currently beta-test code")
                    self.native = mavnative.NativeConnection(MAVLink_message, mavlink_map)
                else:
                    self.native = None
                if native_testing:
                    self.test_buf = bytearray()
                self.mav20_unpacker = struct.Struct('<cBBBBBBHB')
                self.mav10_unpacker = struct.Struct('<cBBBBB')
                self.mav20_h3_unpacker = struct.Struct('BBB')
                self.mav_csum_unpacker = struct.Struct('<H')
                self.mav_sign_unpacker = struct.Struct('<IH')

        def set_callback(self, callback, *args, **kwargs):
            self.callback = callback
            self.callback_args = args
            self.callback_kwargs = kwargs

        def set_send_callback(self, callback, *args, **kwargs):
            self.send_callback = callback
            self.send_callback_args = args
            self.send_callback_kwargs = kwargs

        def send(self, mavmsg, force_mavlink1=False):
                '''send a MAVLink message'''
                buf = mavmsg.pack(self, force_mavlink1=force_mavlink1)
                self.file.write(buf)
                self.seq = (self.seq + 1) % 256
                self.total_packets_sent += 1
                self.total_bytes_sent += len(buf)
                if self.send_callback:
                    self.send_callback(mavmsg, *self.send_callback_args, **self.send_callback_kwargs)

        def buf_len(self):
            return len(self.buf) - self.buf_index

        def bytes_needed(self):
            '''return number of bytes needed for next parsing stage'''
            if self.native:
                ret = self.native.expected_length - self.buf_len()
            else:
                ret = self.expected_length - self.buf_len()

            if ret <= 0:
                return 1
            return ret

        def __parse_char_native(self, c):
            '''this method exists only to see in profiling results'''
            m = self.native.parse_chars(c)
            return m

        def __callbacks(self, msg):
            '''this method exists only to make profiling results easier to read'''
            if self.callback:
                self.callback(msg, *self.callback_args, **self.callback_kwargs)

        def parse_char(self, c):
            '''input some data bytes, possibly returning a new message'''
            self.buf.extend(c)

            self.total_bytes_received += len(c)

            if self.native:
                if native_testing:
                    self.test_buf.extend(c)
                    m = self.__parse_char_native(self.test_buf)
                    m2 = self.__parse_char_legacy()
                    if m2 != m:
                        print("Native: %s\nLegacy: %s\n" % (m, m2))
                        raise Exception('Native vs. Legacy mismatch')
                else:
                    m = self.__parse_char_native(self.buf)
            else:
                m = self.__parse_char_legacy()

            if m is not None:
                self.total_packets_received += 1
                self.__callbacks(m)
            else:
                # XXX The idea here is if we've read something and there's nothing left in
                # the buffer, reset it to 0 which frees the memory
                if self.buf_len() == 0 and self.buf_index != 0:
                    self.buf = bytearray()
                    self.buf_index = 0

            return m

        def __parse_char_legacy(self):
            '''input some data bytes, possibly returning a new message (uses no native code)'''
            header_len = HEADER_LEN_V1
            if self.buf_len() >= 1 and self.buf[self.buf_index] == PROTOCOL_MARKER_V2:
                header_len = HEADER_LEN_V2

            if self.buf_len() >= 1 and self.buf[self.buf_index] != PROTOCOL_MARKER_V1 and self.buf[self.buf_index] != PROTOCOL_MARKER_V2:
                magic = self.buf[self.buf_index]
                self.buf_index += 1
                if self.robust_parsing:
                    m = MAVLink_bad_data(bytearray([magic]), 'Bad prefix')
                    self.expected_length = header_len+2
                    self.total_receive_errors += 1
                    return m
                if self.have_prefix_error:
                    return None
                self.have_prefix_error = True
                self.total_receive_errors += 1
                raise MAVError("invalid MAVLink prefix '%s'" % magic)
            self.have_prefix_error = False
            if self.buf_len() >= 3:
                sbuf = self.buf[self.buf_index:3+self.buf_index]
                if sys.version_info.major < 3:
                    sbuf = str(sbuf)
                (magic, self.expected_length, incompat_flags) = self.mav20_h3_unpacker.unpack(sbuf)
                if magic == PROTOCOL_MARKER_V2 and (incompat_flags & MAVLINK_IFLAG_SIGNED):
                        self.expected_length += MAVLINK_SIGNATURE_BLOCK_LEN
                self.expected_length += header_len + 2
            if self.expected_length >= (header_len+2) and self.buf_len() >= self.expected_length:
                mbuf = array.array('B', self.buf[self.buf_index:self.buf_index+self.expected_length])
                self.buf_index += self.expected_length
                self.expected_length = header_len+2
                if self.robust_parsing:
                    try:
                        if magic == PROTOCOL_MARKER_V2 and (incompat_flags & ~MAVLINK_IFLAG_SIGNED) != 0:
                            raise MAVError('invalid incompat_flags 0x%x 0x%x %u' % (incompat_flags, magic, self.expected_length))
                        m = self.decode(mbuf)
                    except MAVError as reason:
                        m = MAVLink_bad_data(mbuf, reason.message)
                        self.total_receive_errors += 1
                else:
                    if magic == PROTOCOL_MARKER_V2 and (incompat_flags & ~MAVLINK_IFLAG_SIGNED) != 0:
                        raise MAVError('invalid incompat_flags 0x%x 0x%x %u' % (incompat_flags, magic, self.expected_length))
                    m = self.decode(mbuf)
                return m
            return None

        def parse_buffer(self, s):
            '''input some data bytes, possibly returning a list of new messages'''
            m = self.parse_char(s)
            if m is None:
                return None
            ret = [m]
            while True:
                m = self.parse_char("")
                if m is None:
                    return ret
                ret.append(m)
            return ret

        def check_signature(self, msgbuf, srcSystem, srcComponent):
            '''check signature on incoming message'''
            if isinstance(msgbuf, array.array):
                msgbuf = msgbuf.tostring()
            timestamp_buf = msgbuf[-12:-6]
            link_id = msgbuf[-13]
            (tlow, thigh) = self.mav_sign_unpacker.unpack(timestamp_buf)
            timestamp = tlow + (thigh<<32)

            # see if the timestamp is acceptable
            stream_key = (link_id,srcSystem,srcComponent)
            if stream_key in self.signing.stream_timestamps:
                if timestamp <= self.signing.stream_timestamps[stream_key]:
                    # reject old timestamp
                    # print('old timestamp')
                    return False
            else:
                # a new stream has appeared. Accept the timestamp if it is at most
                # one minute behind our current timestamp
                if timestamp + 6000*1000 < self.signing.timestamp:
                    # print('bad new stream ', timestamp/(100.0*1000*60*60*24*365), self.signing.timestamp/(100.0*1000*60*60*24*365))
                    return False
                self.signing.stream_timestamps[stream_key] = timestamp
                # print('new stream')

            h = hashlib.new('sha256')
            h.update(self.signing.secret_key)
            h.update(msgbuf[:-6])
            sig1 = str(h.digest())[:6]
            sig2 = str(msgbuf)[-6:]
            if sig1 != sig2:
                # print('sig mismatch')
                return False

            # the timestamp we next send with is the max of the received timestamp and
            # our current timestamp
            self.signing.timestamp = max(self.signing.timestamp, timestamp)
            return True

        def decode(self, msgbuf):
                '''decode a buffer as a MAVLink message'''
                # decode the header
                if msgbuf[0] != PROTOCOL_MARKER_V1:
                    headerlen = 10
                    try:
                        magic, mlen, incompat_flags, compat_flags, seq, srcSystem, srcComponent, msgIdlow, msgIdhigh = self.mav20_unpacker.unpack(msgbuf[:headerlen])
                    except struct.error as emsg:
                        raise MAVError('Unable to unpack MAVLink header: %s' % emsg)
                    msgId = msgIdlow | (msgIdhigh<<16)
                    mapkey = msgId
                else:
                    headerlen = 6
                    try:
                        magic, mlen, seq, srcSystem, srcComponent, msgId = self.mav10_unpacker.unpack(msgbuf[:headerlen])
                        incompat_flags = 0
                        compat_flags = 0
                    except struct.error as emsg:
                        raise MAVError('Unable to unpack MAVLink header: %s' % emsg)
                    mapkey = msgId
                if (incompat_flags & MAVLINK_IFLAG_SIGNED) != 0:
                    signature_len = MAVLINK_SIGNATURE_BLOCK_LEN
                else:
                    signature_len = 0

                if ord(magic) != PROTOCOL_MARKER_V1 and ord(magic) != PROTOCOL_MARKER_V2:
                    raise MAVError("invalid MAVLink prefix '%s'" % magic)
                if mlen != len(msgbuf)-(headerlen+2+signature_len):
                    raise MAVError('invalid MAVLink message length. Got %u expected %u, msgId=%u headerlen=%u' % (len(msgbuf)-(headerlen+2+signature_len), mlen, msgId, headerlen))

                if not mapkey in mavlink_map:
                    raise MAVError('unknown MAVLink message ID %s' % str(mapkey))

                # decode the payload
                type = mavlink_map[mapkey]
                fmt = type.format
                order_map = type.orders
                len_map = type.lengths
                crc_extra = type.crc_extra

                # decode the checksum
                try:
                    crc, = self.mav_csum_unpacker.unpack(msgbuf[-(2+signature_len):][:2])
                except struct.error as emsg:
                    raise MAVError('Unable to unpack MAVLink CRC: %s' % emsg)
                crcbuf = msgbuf[1:-(2+signature_len)]
                if True: # using CRC extra
                    crcbuf.append(crc_extra)
                crc2 = x25crc(crcbuf)
                if crc != crc2.crc:
                    raise MAVError('invalid MAVLink CRC in msgID %u 0x%04x should be 0x%04x' % (msgId, crc, crc2.crc))

                sig_ok = False
                if signature_len == MAVLINK_SIGNATURE_BLOCK_LEN:
                    self.signing.sig_count += 1
                if self.signing.secret_key is not None:
                    accept_signature = False
                    if signature_len == MAVLINK_SIGNATURE_BLOCK_LEN:
                        sig_ok = self.check_signature(msgbuf, srcSystem, srcComponent)
                        accept_signature = sig_ok
                        if sig_ok:
                            self.signing.goodsig_count += 1
                        else:
                            self.signing.badsig_count += 1
                        if not accept_signature and self.signing.allow_unsigned_callback is not None:
                            accept_signature = self.signing.allow_unsigned_callback(self, msgId)
                            if accept_signature:
                                self.signing.unsigned_count += 1
                            else:
                                self.signing.reject_count += 1
                    elif self.signing.allow_unsigned_callback is not None:
                        accept_signature = self.signing.allow_unsigned_callback(self, msgId)
                        if accept_signature:
                            self.signing.unsigned_count += 1
                        else:
                            self.signing.reject_count += 1
                    if not accept_signature:
                        raise MAVError('Invalid signature')

                csize = type.unpacker.size
                mbuf = msgbuf[headerlen:-(2+signature_len)]
                if len(mbuf) < csize:
                    # zero pad to give right size
                    mbuf.extend([0]*(csize - len(mbuf)))
                if len(mbuf) < csize:
                    raise MAVError('Bad message of type %s length %u needs %s' % (
                        type, len(mbuf), csize))
                mbuf = mbuf[:csize]
                try:
                    t = type.unpacker.unpack(mbuf)
                except struct.error as emsg:
                    raise MAVError('Unable to unpack MAVLink payload type=%s fmt=%s payloadLength=%u: %s' % (
                        type, fmt, len(mbuf), emsg))

                tlist = list(t)
                # handle sorted fields
                if True:
                    t = tlist[:]
                    if sum(len_map) == len(len_map):
                        # message has no arrays in it
                        for i in range(0, len(tlist)):
                            tlist[i] = t[order_map[i]]
                    else:
                        # message has some arrays
                        tlist = []
                        for i in range(0, len(order_map)):
                            order = order_map[i]
                            L = len_map[order]
                            tip = sum(len_map[:order])
                            field = t[tip]
                            if L == 1 or isinstance(field, str):
                                tlist.append(field)
                            else:
                                tlist.append(t[tip:(tip + L)])

                # terminate any strings
                for i in range(0, len(tlist)):
                    if type.fieldtypes[i] == 'char':
                        if sys.version_info.major >= 3:
                            tlist[i] = tlist[i].decode('utf-8')
                        tlist[i] = str(MAVString(tlist[i]))
                t = tuple(tlist)
                # construct the message object
                try:
                    m = type(*t)
                except Exception as emsg:
                    raise MAVError('Unable to instantiate MAVLink message of type %s : %s' % (type, emsg))
                m._signed = sig_ok
                if m._signed:
                    m._link_id = msgbuf[-13]
                m._msgbuf = msgbuf
                m._payload = msgbuf[6:-(2+signature_len)]
                m._crc = crc
                m._header = MAVLink_header(msgId, incompat_flags, compat_flags, mlen, seq, srcSystem, srcComponent)
                return m
        def control_info_encode(self, timestamp, quaternion, position, angular_velocity, velocity, control_h, control_l):
                '''
                state action pair

                timestamp                 : timestamp (type:uint64_t)
                quaternion                : quaternion (type:float)
                position                  : position (type:float)
                angular_velocity          : angular velocity (type:float)
                velocity                  : velocity (type:float)
                control_h                 : high level control (type:float)
                control_l                 : low level control (type:float)

                '''
                return MAVLink_control_info_message(timestamp, quaternion, position, angular_velocity, velocity, control_h, control_l)

        def control_info_send(self, timestamp, quaternion, position, angular_velocity, velocity, control_h, control_l, force_mavlink1=False):
                '''
                state action pair

                timestamp                 : timestamp (type:uint64_t)
                quaternion                : quaternion (type:float)
                position                  : position (type:float)
                angular_velocity          : angular velocity (type:float)
                velocity                  : velocity (type:float)
                control_h                 : high level control (type:float)
                control_l                 : low level control (type:float)

                '''
                return self.send(self.control_info_encode(timestamp, quaternion, position, angular_velocity, velocity, control_h, control_l), force_mavlink1=force_mavlink1)

        def nn_lib_info_encode(self,  D_timestamp, dir):
                '''
                update nn shared obj

                 D_timestamp              : delta t (type:uint64_t)
                dir                       : new directory of nn shared obj (type:char)

                '''
                return MAVLink_nn_lib_info_message( D_timestamp, dir)

        def nn_lib_info_send(self,  D_timestamp, dir, force_mavlink1=False):
                '''
                update nn shared obj

                 D_timestamp              : delta t (type:uint64_t)
                dir                       : new directory of nn shared obj (type:char)

                '''
                return self.send(self.nn_lib_info_encode( D_timestamp, dir), force_mavlink1=force_mavlink1)

