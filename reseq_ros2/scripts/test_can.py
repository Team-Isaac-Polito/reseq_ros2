#!/bin/python3
from time import sleep
from termcolor import colored
import struct
import argparse
import can

freq = 100  # 100Hz as specified in PicoLowLevel

messages = {}
descriptions = {
    0x22: "Motor Feedback",
    0x32: "Joint Yaw Feedback",
    0x42: "End Effector Pitch Feedback",
    0x44: "End Effector Head Pitch Feedback",
    0x46: "End Effector Head Roll Feedback",
}

# Currently sent feedbacks (from PicoLowLevel)
common = [0x22]
joint = [0x32]
end_effector = [0x42, 0x44, 0x46]


def can_callback(msg):
    # Decodification of message (view communication node)
    decoded_aid = struct.unpack("4b", msg.arbitration_id.to_bytes(4, "big"))
    module_id = decoded_aid[3]
    packet_id = decoded_aid[1]

    # Keeps a counter for the messages
    messages[module_id][packet_id] += 1


def print_results(t):
    # Used to color the output for readability
    d = {True: colored("OK", "green"), False: colored("NO", "red")}
    dm = {True: colored("CONNECTED", "green"), False: colored("DISCONNECTED", "red")}
    fq = lambda x: (
        colored("OK", "green") if x > (freq * 0.98) else colored("SLOW", "red")
    )

    all_received = True
    for addr in messages.keys():
        module_connected = False
        print(f"Module: {hex(addr)}")
        for msg_id, received in messages[addr].items():
            print(
                f"+ {descriptions[msg_id]} ({hex(msg_id)}): {d[bool(received)]} | {received/t} Hz {fq(received/t)}"
            )
            if received != 0:
                module_connected = True
            else:
                all_received = False
        print(f"MODULE: {dm[module_connected]}\n")
    return all_received


def verify_feedback(reseq=1, modules_number=2, can_channel="can0", t=5):

    for i in range(1, modules_number + 1):
        module_id = 0x10 * reseq + i
        mod_msg = {msg_id: 0 for msg_id in common}
        if i == 1:
            mod_msg.update({msg_id: 0 for msg_id in end_effector})
        else:
            mod_msg.update({msg_id: 0 for msg_id in joint})
        messages[module_id] = mod_msg

    canbus = can.interface.Bus(channel=can_channel, bustype="socketcan")
    notifier = can.Notifier(canbus, [can_callback])

    # Simple progress bar to show that the program is not freezed
    print("Verifying CAN messages", end="", flush=True)
    for _ in range(t):
        print(".", end="", flush=True)
        sleep(1)
    print("\n")

    res = print_results(t)

    notifier.stop()
    canbus.shutdown()

    return res


def main():
    parser = argparse.ArgumentParser(
        description="""
            Verifies CAN connections to Rese.Q modules, that feedback messages are received and checks their frequency.
            If all messages are received, exits with the success code (independently from their frequency)
            """
    )

    parser.add_argument("--reseq", type=int, help="Rese.Q MK version", default=1)
    parser.add_argument("--modules", type=int, help="Number of modules", default=2)
    parser.add_argument("--can", type=str, help="CAN channel", default="can0")
    parser.add_argument("--time", type=int, help="Duration of verification", default=5)

    args = parser.parse_args()
    res = verify_feedback(
        reseq=args.reseq, modules_number=args.modules, can_channel=args.can, t=args.time
    )

    exit(0 if res else 1)


if __name__ == "__main__":
    main()
