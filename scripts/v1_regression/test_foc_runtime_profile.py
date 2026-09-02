import unittest
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))

from common.foc_runtime_profile import (
    FocSerial,
    MixedStreamDecoder,
    crc8_poly07,
)


def make_current_frame(seed: int = 0) -> bytes:
    body = bytes([0xA5, 0x5A, 0x43, 20]) + bytes(
        ((seed + index) & 0xFF) for index in range(20)
    )
    return body + bytes([crc8_poly07(body)])


class FakeSerial:
    def __init__(self, initial: bytes = b"", responses=None) -> None:
        self.rx = bytearray(initial)
        self.responses = responses or {}
        self.writes = []
        self.reset_calls = 0

    @property
    def in_waiting(self) -> int:
        return len(self.rx)

    def read(self, size: int = 1) -> bytes:
        if not self.rx:
            return b""
        count = min(size, len(self.rx))
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data

    def write(self, data: bytes) -> int:
        command = data.decode("ascii").strip()
        self.writes.append(command)
        self.rx.extend(self.responses.get(command, b""))
        return len(data)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        self.reset_calls += 1
        self.rx.clear()


class MixedStreamDecoderTests(unittest.TestCase):
    def test_split_binary_frame_and_ascii_lines(self) -> None:
        decoder = MixedStreamDecoder()
        frame = make_current_frame(7)

        first = decoder.feed(b"N,old\r\n" + frame[:9])
        second = decoder.feed(frame[9:] + b"UART_RX,OK,err=0\r\n")

        self.assertEqual(first, ["N,old"])
        self.assertEqual(second, ["UART_RX,OK,err=0"])
        self.assertEqual(decoder.binary_frames, 1)
        self.assertEqual(decoder.binary_crc_errors, 0)


class FocSerialCommandTests(unittest.TestCase):
    def test_command_decodes_stale_input_without_matching_it(self) -> None:
        frame = make_current_frame(3)
        fake = FakeSerial(
            initial=b"FW_INFO,OK,stale\r\n" + frame,
            responses={"CMD:FW_INFO?": b"FW_INFO,OK,new\r\n"},
        )
        link = FocSerial("TEST", 1_000_000, serial_port=fake)

        lines = link.command("CMD:FW_INFO?", "FW_INFO,OK")

        self.assertEqual(lines, ["FW_INFO,OK,new"])
        self.assertIn("FW_INFO,OK,stale", link.all_lines)
        self.assertEqual(link.decoder.binary_frames, 1)
        self.assertEqual(link.decoder.binary_crc_errors, 0)
        self.assertEqual(fake.reset_calls, 0)

    def test_command_uses_response_prefix_not_substring(self) -> None:
        fake = FakeSerial(
            responses={
                "CMD:FW_INFO?": (
                    b"N,text_contains_FW_INFO,OK\r\n"
                    b"FW_INFO,OK,version=1.0.0\r\n"
                )
            }
        )
        link = FocSerial("TEST", 1_000_000, serial_port=fake)

        lines = link.command("CMD:FW_INFO?", "FW_INFO,OK")

        self.assertEqual(lines[-1], "FW_INFO,OK,version=1.0.0")
        self.assertEqual(fake.reset_calls, 0)

    def test_uart_stats_uses_canonical_diag_command(self) -> None:
        response = (
            b"UART_RX,OK,err=2,restart_fail=1,last_pos=8,buf=256,"
            b"tx_p0_drop=3,tx_p1_drop=4,tx_p2_drop=5\r\n"
        )
        fake = FakeSerial(responses={"DIAG:UART_RX?": response})
        link = FocSerial("TEST", 1_000_000, serial_port=fake)

        stats = link.query_uart_stats()

        self.assertEqual(fake.writes, ["DIAG:UART_RX?"])
        self.assertEqual(stats.rx_errors, 2)
        self.assertEqual(stats.restart_failures, 1)
        self.assertEqual(stats.tx_p0_drops, 3)
        self.assertEqual(stats.tx_p1_drops, 4)
        self.assertEqual(stats.tx_p2_drops, 5)
        self.assertEqual(fake.reset_calls, 0)

    def test_foc_time_requires_new_begin_before_end(self) -> None:
        fake = FakeSerial(
            initial=b"FOC_TIME,END,jitter_us=999\r\n",
            responses={
                "DIAG:FOC_TIME?": (
                    b"FOC_TIME,BEGIN,dwt=1\r\n"
                    b"FOC_TIME,TIM1_ISR,n=10,max_us=20\r\n"
                    b"FOC_TIME,END,jitter_us=1\r\n"
                )
            },
        )
        link = FocSerial("TEST", 1_000_000, serial_port=fake)

        lines = link.query_foc_time()

        self.assertEqual(lines[0], "FOC_TIME,BEGIN,dwt=1")
        self.assertEqual(lines[-1], "FOC_TIME,END,jitter_us=1")
        self.assertEqual(len(lines), 3)
        self.assertEqual(fake.reset_calls, 0)

    def test_timeout_collects_uart_drop_evidence_without_retry(self) -> None:
        response = (
            b"UART_RX,OK,err=0,restart_fail=0,last_pos=8,buf=256,"
            b"tx_p0_drop=2,tx_p1_drop=0,tx_p2_drop=0\r\n"
        )
        fake = FakeSerial(responses={"DIAG:UART_RX?": response})
        link = FocSerial("TEST", 1_000_000, serial_port=fake)
        link.last_uart_stats = link._query_uart_stats_best_effort()
        fake.responses["DIAG:UART_RX?"] = response.replace(b"tx_p0_drop=2", b"tx_p0_drop=4")

        with self.assertRaisesRegex(RuntimeError, r"tx_p0_drops=2"):
            link.command("CMD:ENABLE,1", "ENABLE,OK,1", timeout=0.001)

        self.assertEqual(fake.writes.count("CMD:ENABLE,1"), 1)
        self.assertEqual(fake.writes[-1], "DIAG:UART_RX?")
        self.assertEqual(fake.reset_calls, 0)


if __name__ == "__main__":
    unittest.main()
