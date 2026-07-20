## Buggy-Server Communication Protocol
### Custom ROS2 Message Layout
All communication between the Buggy and the Server is handled through a custom ROS2 message.
Message Format file location: cognipilot/cranium/src/synapse_msgs/msg/ServerCommunication.msg
```msg
uint8 src         # Who sent it? (Buggy = 1, Server = 2)
uint8 dest        # Who is it for? (Buggy = 1, Server = 2)
uint8 uid         # Message ID / Roll counter to keep track of conversations
uint8 ack         # Status flag (0 for standard data, 1 to acknowledge receipt)
string msg        # The main payload buffer for QR text or mission updates
```

#### Field-by-Field Breakdown

* **`src` (`uint8`)**: Identifies the sender. When your buggy sends a message, set this to `1`. The server will use `2`.
* **`dest` (`uint8`)**: Identifies the intended recipient. Your buggy node should look out for packets addressed to `1`, and ignore anything else!
* **`uid` (`uint8`)**: A rolling sequence tracker (`0` to `255`). This helps both sides match the right acknowledgment to the right message. The Buggy and Server keep track of their own independent counters.
* **`ack` (`uint8`)**: The confirmation flag. Setting this to `1` lets the other side know you successfully received their message with that exact `uid`.
* **`msg` (`string`)**: The dynamic text payload. This carries raw scanned QR strings, status keywords like `"PARKED"` or `"INVALID"`, or stays blank (`""`) during quick structural receipts.

---

### 🔄 The Navigation Workflow & Handshake Sequence

Your communication lifecycle flows through three main phases as your buggy completes its route:

```text
    🤖 Buggy (ID: 1)                                     💻 Server (ID: 2)
          |                                                     |
          | -------- [src=1, dest=2, uid=10, msg="A"] --------> |  (1. Scanned QR Patient)
          | <---- [src=2, dest=1, uid=10, ack=1, msg=""] ------ |  (2. Quick acknowledgment)
          |                                                     |
          |                      [ Brief Validation Delay ]     |
          |                                                     |
          | <------- [src=2, dest=1, uid=101, msg="X"] -------- |  (3. Next Target - Hospital)
          | ----- [src=1, dest=2, uid=101, ack=1, msg=""] ----> |  (4. Target Confirmed)
          |                                                     |
```
#### 1. Patient/Hospital Arrival Message
Whenever your buggy successfully scans a patient/hospital QR and is in the designated area, it should immediately text the server. Send the exact text from the QR code inside the `msg` field. The server will respond instantly by sending back an empty acknowledgment message (`msg = ""` and `ack = 1`) matching your `uid`.

#### 2. Server Destination Update
After the server double-checks where your buggy is on the map, it will pause for a brief moment to process, then send out your next destination target (like `"B"` or `"X"` which corresponds to different patient or hospital names). Your buggy should after reading this message should instantly send back a matching `ack = 1` packet to confirm the message and head to the next destination.

#### 3. Bonus (Parking)
Once all deliveries are finished and your buggy has successfully navigated to the last hospital, it needs to park in the parking ot next to the hospital. After parking, send a final `msg = "PARKED"` payload to successfully wrap up your official run. Wait for response from server as `msg = "OK"` to confirm that your parking is correct. If server sends the message `msg = "INVALID"`, it means that buggy is not parked correctly.

---

### Important Notes:

> ℹ️ **Malformed Packets**
> If a message gets scrambled in transit (fails to unpack, misses vital fields, or overflows limits), don't worry—the server will simply ignore it. It won't count as a failed attempt. Server will only look at mesages where the src and dest are correctly filled.

> ⚠️ **Positional & QR Accuracy**
> If your buggy sends a structurally perfect message but the QR text doesn't match the target, or if it fires the message while physically sitting *outside* the valid coordinate zone for that waypoint, the server will reply with `msg = "INVALID"`, and **marks will be deducted** from your team's score.

> ⚠️ **ACK messages**
> If the server sends out a new mission target but doesn't hear an acknowledgment (`ack = 1`) back from your buggy within its timing window, it will retry transmission **up to 5 additional times**. If there's still no response after the 5th retry, the system will assume a "Connection Lost" state and safely stop the run.