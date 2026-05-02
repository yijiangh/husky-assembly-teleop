import serial
import time
import statistics

PORT = "/tmp/ttyUR"
BAUD = 115200
TIMEOUT = 1.0   # seconds

def open_port():
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT
    )
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.1)
    return ser

def send_cmd(ser, cmd):
    try:
        ser.write((cmd + "\n").encode("ascii"))
        ser.flush()
        reply = ser.readline().decode("ascii").strip()
        return reply
    except Exception as e:
        return f"EXCEPTION: {e}"

def test_step(ser, description, cmd, expect_prefix=None):
    reply = send_cmd(ser, cmd)
    ok = True

    if reply == "":
        ok = False
    elif expect_prefix and not reply.startswith(expect_prefix):
        ok = False

    status = "OK" if ok else "FAIL"
    print(f"[{status}] {description}")
    print(f"  CMD: {cmd}")
    print(f"  RSP: {reply}")
    return ok

def main():
    ser = open_port()
    print("Serial port opened.\n")

    # -----------------------------
    # Basic connectivity
    # -----------------------------
    test_step(ser, "Ping test", "PING", "OK")

    # -----------------------------
    # SET / GET parameters
    # -----------------------------
    test_step(ser, "Set gripper open pos", "SET_GRIPPER_OPEN_POS 1100", "OK")
    test_step(ser, "Get gripper open pos", "GET_GRIPPER_OPEN_POS", "OK")

    test_step(ser, "Set gripper close pos", "SET_GRIPPER_CLOSE_POS 1900", "OK")
    test_step(ser, "Get gripper close pos", "GET_GRIPPER_CLOSE_POS", "OK")

    test_step(ser, "Set screw max current", "SET_SCREW_MAX_CURRENT 900", "OK")
    test_step(ser, "Get screw max current", "GET_SCREW_MAX_CURRENT", "OK")

    test_step(ser, "Set screw speed", "SET_SCREW_SPEED 180", "OK")
    test_step(ser, "Get screw speed", "GET_SCREW_SPEED", "OK")

    test_step(ser, "Set stall time", "SET_SCREW_STALL_TIME 400", "OK")
    test_step(ser, "Get stall time", "GET_SCREW_STALL_TIME", "OK")

    test_step(ser, "Set stall drop", "SET_SCREW_STALL_DROP 35", "OK")
    test_step(ser, "Get stall drop", "GET_SCREW_STALL_DROP", "OK")

    # -----------------------------
    # Action commands
    # -----------------------------
    test_step(ser, "Open gripper", "OPEN_GRIPPER", "OK")
    test_step(ser, "Close gripper", "CLOSE_GRIPPER", "OK")

    test_step(ser, "Start tighten", "START_TIGHTEN", "OK")
    time.sleep(1.0)
    test_step(ser, "Report status (during tighten)", "REPORT_STATUS", "OK")

    time.sleep(6.0)  # allow fake stall
    test_step(ser, "Report status (after stall)", "REPORT_STATUS", "OK")

    test_step(ser, "Stop screw", "STOP_SCREW", "OK")
    test_step(ser, "Report status (stopped)", "REPORT_STATUS", "OK")

    test_step(ser, "Start loosen", "START_LOOSEN", "OK")
    test_step(ser, "Report status (loosen)", "REPORT_STATUS", "OK")

    # -----------------------------
    # Latency test (PING–PONG)
    # -----------------------------
    print("\nLatency test (20 PINGs):")
    latencies_ms = []

    for i in range(20):
        t0 = time.perf_counter()
        reply = send_cmd(ser, "PING")
        t1 = time.perf_counter()

        if reply.startswith("OK"):
            dt_ms = (t1 - t0) * 1000.0
            latencies_ms.append(dt_ms)
            print(f"  {i+1:02d}: {dt_ms:.2f} ms")
        else:
            print(f"  {i+1:02d}: FAIL ({reply})")

        time.sleep(0.1)

    if latencies_ms:
        print("\nLatency summary:")
        print(f"  min: {min(latencies_ms):.2f} ms")
        print(f"  max: {max(latencies_ms):.2f} ms")
        print(f"  avg: {statistics.mean(latencies_ms):.2f} ms")

    ser.close()
    print("\nDone.")

if __name__ == "__main__":
    main()