"""Transport to the operator: MQTT in one direction, JSON in both.

Inbound is a single mailbox holding the newest request, consumed on read. The
operator is one person pressing one button at a time, so there is nothing to
queue - and a queue would be worse than nothing, since an abort waiting behind
a stale start is an abort that arrives late.

Everything here runs on paho's network thread except poll(), which runs on the
commander's tick. The lock covers exactly that handoff and nothing else.

Connecting is asynchronous on purpose: a broker that is down must not stall
node startup, and the subscription is made in on_connect so it is re-made on
every reconnect rather than only the first one.
"""

import dataclasses
import json
import threading

import paho.mqtt.client as mqtt

BROKER_HOST = 'localhost'
BROKER_PORT = 1883
# The node's identity on the broker, not the topic root. Distinct from the
# page's client so the broker cannot mistake them for the same connection and
# evict one - MQTT allows a client id exactly once.
CLIENT_ID = 'udl_aa_fc'

TOPIC_CMD = 'udl_aa_gcs/cmd'
TOPIC_TELEMETRY = 'udl_aa_gcs/telemetry'
TOPIC_LOG = 'udl_aa_gcs/log'


START = 'start'
STOP = 'stop'
ABORT = 'abort'


@dataclasses.dataclass
class GuiRequest:
    """One operator action.

    START runs `name`, optionally with `params` - the d-pad carries its axis
    that way. STOP ends whatever is running and starts nothing, which is what a
    released d-pad button sends. ABORT ends it and hands over to the abort
    sequence. Only START uses `name` and `params`.
    """

    action: str
    name: str
    params: dict


class GuiLink:

    def __init__(self):
        self._lock = threading.Lock()
        self._latest = None

        self._client = _new_client()
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.connect_async(BROKER_HOST, BROKER_PORT)
        self._client.loop_start()

    def close(self):
        self._client.loop_stop()
        self._client.disconnect()

    # ------------------------------------------------------------------
    # paho's thread - touches nothing but the mailbox
    # ------------------------------------------------------------------

    def _on_connect(self, client, userdata, flags, rc):
        client.subscribe(TOPIC_CMD, qos=1)

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
        except (ValueError, UnicodeDecodeError):
            return

        action = payload.get('action')
        if action not in (START, STOP, ABORT):
            return
        request = GuiRequest(action,
                             payload.get('name', ''),
                             payload.get('params') or {})

        with self._lock:
            self._latest = request   # overwrite: newest wins

    # ------------------------------------------------------------------
    # The commander's tick
    # ------------------------------------------------------------------

    def poll(self):
        """The newest request, or None. Consumed on read."""
        with self._lock:
            request, self._latest = self._latest, None
        return request

    def publish_telemetry(self, packet):
        """QoS 0: a dropped frame is replaced 200 ms later by a fresher one."""
        self._client.publish(TOPIC_TELEMETRY, json.dumps(packet), qos=0)

    def publish_log(self, text):
        """QoS 1: log lines are the record of what happened, not a snapshot."""
        self._client.publish(TOPIC_LOG, json.dumps({'text': text}), qos=1)


def _new_client():
    """paho 2.x requires a callback API version; 1.x does not accept one.

    Pinned to VERSION1 so the callback signatures above are the same either
    way, rather than depending on which of the two is installed.
    """
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=CLIENT_ID)
    except AttributeError:
        return mqtt.Client(client_id=CLIENT_ID)
