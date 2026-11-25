#!/usr/bin/env python3
"""Watch the robots' Wi-Fi live: ping them, log and plot every ping, and pop up a small window when one is slow.

    python src/husky-assembly-teleop/scripts/wifi_watch.py
    python src/husky-assembly-teleop/scripts/wifi_watch.py --limit 300 --minutes 5

Needs a lab address on this PC, an ssh key on the robots, a screen and Tk (`sudo apt install python3-tk`); no ROS.
Everything is kept in `~/.ros/wifi_watch/<YYYYMMDD>/<HHMMSS>/`: `ping.csv` (every ping; none while a host is off),
`link.csv` (each robot's radio every 2 s: channel, signal, resends), `events.log` (slow spells, channel changes) and
`plot.png` (the window, every minute and at the end).
The robots are only read. The popup never takes the keyboard focus; a click closes it until the next slow spell.
"""

from __future__ import annotations

import argparse
import csv
import re
import signal
import statistics
import subprocess
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from os import nice
from pathlib import Path

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.dates import date2num, DateFormatter
from matplotlib.figure import Figure

ROUTER = "router"
#: Names and lab addresses; every host but the router is a robot, read over ssh as well (scripts/real_lab/env.sh).
HOSTS = {ROUTER: "192.168.0.1", "alice": "192.168.0.113", "belle": "192.168.0.114", "cindy": "192.168.0.115"}
COLORS = {ROUTER: "#8a8984", "alice": "#2a78d6", "belle": "#eb6834", "cindy": "#1baf7a"}

#: Seconds between pings, and between the readings of a robot's radio.
PING_PERIOD = 0.2
LINK_PERIOD = 2.0
#: Seconds without a slow ping after which a slow spell is over and the popup closes.
SPELL_HOLD = 10.0
#: Seconds without an answer after which a host counts as switched off, not slow: no popup, nothing logged.
OFF_AFTER = 60.0
#: Seconds before a ping or ssh that ended is started again.
RESTART_DELAY = 5.0
#: Seconds the numbers beside the plots look back.
RECENT = 10.0

#: What is read out of `iw dev wlp2s0 info` and `iw dev wlp2s0 station dump`.
LINK_PATTERNS = {"channel": r"channel ([^\n]+)", "signal": r"signal:\s*(-?\d+ \[[^\]]*\]|-?\d+)",
                 "tx_packets": r"tx packets:\s*(\d+)", "tx_retries": r"tx retries:\s*(\d+)",
                 "tx_failed": r"tx failed:\s*(\d+)", "rx_packets": r"rx packets:\s*(\d+)",
                 "tx_bytes": r"tx bytes:\s*(\d+)", "rx_bytes": r"rx bytes:\s*(\d+)",
                 "beacon_loss": r"beacon loss:\s*(\d+)", "tx_rate": r"tx bitrate:\s*([^\n]+)",
                 "rx_rate": r"rx bitrate:\s*([^\n]+)"}
LINK_COMMAND = ("while :; do iw dev wlp2s0 info | grep channel; iw dev wlp2s0 station dump; echo ==; "
                f"sleep {LINK_PERIOD:g}; done")


class Spell:
    """A host's slow spell: from its first slow ping until none was slow for SPELL_HOLD."""

    def __init__(self, start: float) -> None:
        """Start a spell at `start`, wall time."""
        self.start = start
        self.last = start
        self.count = 0
        self.worst = 0.0


class Watch:
    """Pings the hosts and reads the robots' radios in threads, keeps what they say and logs it."""

    def __init__(self, folder: Path, limit: float, keep: float) -> None:
        """Log to `folder`; pings over `limit` ms are slow; keep the last `keep` seconds for the plot."""
        self.limit = limit
        self.lock = threading.Lock()
        self.pings = {host: deque(maxlen=int(keep / PING_PERIOD)) for host in HOSTS}  # (sent, ms)
        self.last_answer = {host: 0.0 for host in HOSTS}
        self.spells: dict[str, Spell] = {}
        self.channels: dict[str, str] = {}
        self.channel_changes: list[tuple[float, str, str]] = []  # (time, host, channel)
        self.signals: dict[str, str] = {}
        self._stopped = False
        self._processes: dict[str, subprocess.Popen] = {}
        self._ping_file = open(folder / "ping.csv", "w", newline="")
        self._link_file = open(folder / "link.csv", "w", newline="")
        self._event_file = open(folder / "events.log", "w")
        self._ping_csv, self._link_csv = csv.writer(self._ping_file), csv.writer(self._link_file)
        self._ping_csv.writerow(["t", "host", "seq", "rtt_ms"])
        self._link_csv.writerow(["t", "host", *LINK_PATTERNS])

    def start(self) -> None:
        """Start a ping per host, and a radio reading per robot."""
        for host in HOSTS:
            threading.Thread(target=self._watch_pings, args=(host,), daemon=True).start()
            if host != ROUTER:
                threading.Thread(target=self._watch_link, args=(host,), daemon=True).start()

    def stop(self) -> None:
        """End the pings and ssh connections and close the logs."""
        self._stopped = True
        for process in list(self._processes.values()):
            process.terminate()
        with self.lock:
            for file in (self._ping_file, self._link_file, self._event_file):
                file.close()

    def waiting(self, host: str, now: float) -> float:
        """Return the milliseconds the host's oldest unanswered ping has waited, 0 if none, or never answered."""
        last = self.last_answer[host]
        return max(0.0, (now - last - PING_PERIOD) * 1e3) if last else 0.0

    def check_spells(self, now: float) -> None:
        """Count a ping still waiting as slow, end the spells that are over, and write the logs out."""
        with self.lock:
            for host in HOSTS:
                waited = self.waiting(host, now)
                if self.limit < waited < OFF_AFTER * 1e3:
                    self._slow(host, waited, now, counts=False)
            for host, spell in list(self.spells.items()):
                if now - spell.last > SPELL_HOLD:
                    del self.spells[host]
                    self._event(spell.last, f"{host}: slow spell over, {spell.last - spell.start:.1f} s, worst "
                                            f"{spell.worst:.0f} ms, {spell.count} pings over {self.limit:g} ms")
            for file in (self._ping_file, self._link_file):
                file.flush()

    def _slow(self, host: str, ms: float, now: float, counts: bool = True) -> None:
        spell = self.spells.get(host)
        if spell is None:
            spell = self.spells[host] = Spell(now)
            self._event(now, f"{host}: slow spell starts, {ms:.0f} ms")
        spell.last, spell.worst, spell.count = now, max(spell.worst, ms), spell.count + counts

    def _event(self, t: float, text: str) -> None:
        line = f"{datetime.fromtimestamp(t):%H:%M:%S} {text}"
        print(line, flush=True)
        self._event_file.write(line + "\n")
        self._event_file.flush()

    def _lines(self, key: str, command: list[str]):
        """Yield the command's output lines, starting it again when it ends."""
        while not self._stopped:
            # A session of its own: a Ctrl-C here would end the pings first, which then log as slow.
            process = self._processes[key] = subprocess.Popen(command, stdin=subprocess.DEVNULL, text=True,
                                                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                                              start_new_session=True)
            yield from process.stdout
            time.sleep(RESTART_DELAY)

    def _watch_pings(self, host: str) -> None:
        # With -O a ping unanswered after one period says so ("late" in the log); its answer may still come.
        for line in self._lines(f"ping {host}", ["ping", "-n", "-D", "-O", "-i", str(PING_PERIOD), HOSTS[host]]):
            seq, rtt = re.search(r"icmp_seq=(\d+)", line), re.search(r"time=([\d.]+)", line)
            if not seq or self._stopped:
                continue
            now = time.time()
            with self.lock:
                if self._ping_file.closed:
                    return
                self._log_ping(host, seq.group(1), rtt.group(1) if rtt else None, now)

    def _log_ping(self, host: str, seq: str, rtt: str | None, now: float) -> None:
        """Log a ping answered after `rtt` ms, or still unanswered (None); unless its host is off."""
        if rtt is None and now - self.last_answer[host] > OFF_AFTER:
            return
        self._ping_csv.writerow([f"{now:.3f}", host, seq, rtt or "late"])
        if rtt is None:
            return
        ms = float(rtt)
        self.pings[host].append((now - ms / 1e3, ms))
        self.last_answer[host] = now
        if ms > self.limit:
            self._slow(host, ms, now)

    def _watch_link(self, host: str) -> None:
        # A forced terminal (-tt) ends the loop on the robot when this connection ends.
        command = ["ssh", "-tt", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=5",
                   "-o", "ServerAliveCountMax=2", f"administrator@{HOSTS[host]}", LINK_COMMAND]
        block = ""
        for line in self._lines(f"link {host}", command):
            if not line.startswith("=="):
                block += line.replace("\r", "")
                continue
            found = {key: re.search(pattern, block) for key, pattern in LINK_PATTERNS.items()}
            row = {key: match.group(1).strip() if match else "" for key, match in found.items()}
            block = ""
            now = time.time()
            with self.lock:
                if self._link_file.closed:
                    return
                self._link_csv.writerow([f"{now:.3f}", host, *row.values()])
                self.signals[host] = row["signal"]
                if row["channel"] and row["channel"] != self.channels.get(host):
                    self.channels[host] = row["channel"]
                    self.channel_changes.append((now, host, row["channel"]))
                    self._event(now, f"{host}: channel {row['channel']}")


class Window:
    """The live plot, and the popup while a host is slow."""

    def __init__(self, watch: Watch, folder: Path, keep: float) -> None:
        """Show the last `keep` seconds of `watch`, redrawn every second, and save the plot to `folder`."""
        self._watch, self._folder, self._keep = watch, folder, keep
        self._saved = time.time()
        self._dismissed: set[str] = set()  # hosts whose spell the operator clicked away
        self.root = tk.Tk()
        self.root.title(f"Robot Wi-Fi pings ({folder})")
        # Sizes in points follow the screen's scale; an inch is 72 of them.
        self._inch = self.root.winfo_fpixels("1i")
        figure = Figure(figsize=(10, 8), dpi=self._inch, facecolor="#fcfcfb")
        self._figure = figure
        axes = figure.subplots(len(HOSTS), 1, sharex=True)
        figure.subplots_adjust(left=0.07, right=0.985, top=0.95, bottom=0.05, hspace=0.28)
        self._dots, self._titles, self._axes = {}, {}, dict(zip(HOSTS, axes))
        self._change_marks = []
        for host, ax in self._axes.items():
            ax.set_facecolor("#fcfcfb")
            ax.set_yscale("log")
            ax.set_ylim(0.1, 10000)
            ax.set_yticks([0.1, 1, 10, 100, 1000], ["0.1", "1", "10", "100", "1000"])
            ax.minorticks_off()
            ax.set_ylabel("ms")
            ax.grid(axis="y", color="#e4e3df", lw=0.8)
            for spine in ax.spines.values():
                spine.set_visible(False)
            ax.axhline(watch.limit, color="#52514e", lw=1, ls=(0, (4, 3)))
            self._dots[host], = ax.plot([], [], "o", ms=2.5, mew=0, color=COLORS[host], alpha=0.6)
            self._titles[host] = ax.set_title(host, loc="left", fontsize=10)
        self._axes[list(HOSTS)[-1]].xaxis.set_major_formatter(DateFormatter("%H:%M:%S"))
        self._canvas = FigureCanvasTkAgg(figure, self.root)
        self._canvas.get_tk_widget().pack(fill="both", expand=True)

        # ! Without a window frame it never takes the focus from the operator's monitor page.
        self._popup = tk.Toplevel(self.root, bg="#b3261e")
        self._popup.overrideredirect(True)
        self._popup.attributes("-topmost", True)
        self._popup_text = tk.StringVar()
        label = tk.Label(self._popup, textvariable=self._popup_text, justify="left", anchor="nw", fg="white",
                         bg="#b3261e", font=("DejaVu Sans", 11), padx=12, pady=8)
        label.pack(fill="both", expand=True)
        label.bind("<Button-1>", self._dismiss)
        self._popup.withdraw()
        self.root.after(200, self._tick)

    def save(self) -> None:
        """Save the plot as it shows now."""
        self._figure.savefig(self._folder / "plot.png", dpi=110)

    def _dismiss(self, _event: tk.Event) -> None:
        self._dismissed |= set(self._watch.spells)
        self._popup.withdraw()

    def _tick(self) -> None:
        now = time.time()
        self._watch.check_spells(now)
        self._draw(now)
        self._show_popup(now)
        if now - self._saved > 60.0:
            self._saved = now
            self.save()
        self.root.after(1000, self._tick)

    def _draw(self, now: float) -> None:
        watch = self._watch
        with watch.lock:
            pings = {host: list(values) for host, values in watch.pings.items()}
            changes = list(watch.channel_changes)
            channels, signals = dict(watch.channels), dict(watch.signals)
        for host, values in pings.items():
            self._dots[host].set_data([date2num(datetime.fromtimestamp(t)) for t, _ in values],
                                      [ms for _, ms in values])
            recent = sorted(ms for t, ms in values if now - t < RECENT)
            text = "no answer"
            if recent:
                text = (f"last {RECENT:g} s: median {statistics.median(recent):.1f} ms, 95% under "
                        f"{recent[int(0.95 * (len(recent) - 1))]:.0f}, worst {recent[-1]:.0f}")
            if values:
                slow = sum(ms > watch.limit for _, ms in values)
                text += f"   |   shown: {100 * slow / len(values):.1f}% over {watch.limit:g} ms"
            if host in channels:
                text += f"   |   channel {channels[host].split(' (')[0]}, {signals[host]} dBm"
            if watch.waiting(host, now) > watch.limit:
                text += f"   |   waiting {watch.waiting(host, now) / 1e3:.1f} s"
            self._titles[host].set_text(f"{host.capitalize()}   {text}")
        for line in self._change_marks:
            line.remove()
        self._change_marks = [self._axes[host].axvline(date2num(datetime.fromtimestamp(t)), color="#6250d6", lw=1.5)
                              for t, host, _ in changes if now - t < self._keep]
        self._axes[ROUTER].set_xlim(date2num(datetime.fromtimestamp(now - self._keep)),
                                    date2num(datetime.fromtimestamp(now)))
        self._canvas.draw()

    def _show_popup(self, now: float) -> None:
        with self._watch.lock:
            spells = dict(self._watch.spells)
        self._dismissed &= set(spells)
        if not set(spells) - self._dismissed:
            self._popup.withdraw()
            return
        lines = [f"Slow Wi-Fi (over {self._watch.limit:g} ms)"]
        for host in HOSTS:
            if host in spells:
                spell = spells[host]
                waited = self._watch.waiting(host, now)
                state = f"waiting {waited / 1e3:.1f} s" if waited > self._watch.limit else f"{spell.count} slow"
                lines.append(f"{host.capitalize()}: worst {spell.worst:.0f} ms, {state}")
        # A line per host always, so the popup keeps its size.
        self._popup_text.set("\n".join(lines + [""] * (len(HOSTS) + 1 - len(lines))))
        width, height = int(4.2 * self._inch), int(1.35 * self._inch)
        self._popup.geometry(f"{width}x{height}+{self.root.winfo_screenwidth() - width - 20}+60")
        self._popup.deiconify()
        self._popup.lift()


def main() -> None:
    """Run the watch until the window closes or Ctrl-C."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--limit", type=float, default=200.0,
                        help="ms above which a ping is slow and the popup shows (default %(default)s)")
    parser.add_argument("--minutes", type=float, default=10.0, help="minutes the plot shows (default %(default)s)")
    args = parser.parse_args()
    folder = Path.home() / ".ros" / "wifi_watch" / f"{datetime.now():%Y%m%d}" / f"{datetime.now():%H%M%S}"
    folder.mkdir(parents=True)
    print(f"Logging to {folder}", flush=True)

    watch = Watch(folder, args.limit, args.minutes * 60.0)
    watch.start()
    nice(10)  # The plot gives way to the monitor; the pings, started before, do not.
    window = Window(watch, folder, args.minutes * 60.0)
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: window.root.quit())
    try:
        window.root.mainloop()
    finally:
        watch.stop()
        window.save()


if __name__ == "__main__":
    main()
