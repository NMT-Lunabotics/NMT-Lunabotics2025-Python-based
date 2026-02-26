def discover_robot(
    attempts: int = DISCOVERY_ATTEMPTS, timeout: float = DISCOVERY_TIMEOUT
) -> Optional[Tuple[str, int, int, int]]:
    """Broadcast a discovery packet and wait for f310_comp to respond."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(timeout)
        for _ in range(max(1, attempts)):
            try:
                sock.sendto(DISCOVERY_MAGIC, ("255.255.255.255", DISCOVERY_PORT))
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                payload = json.loads(data.decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
            if not isinstance(payload, dict):
                continue
            if payload.get("role") != "f310_comp":
                continue
            host = addr[0]
            udp_port = int(payload.get("udp_port", UDP_DESTINATION[1]))
            cmd_port = int(payload.get("command_port", COMMAND_DESTINATION[1]))
            telemetry_port = int(payload.get("telemetry_port", TELEMETRY_LISTEN[1]))
            return host, udp_port, cmd_port, telemetry_port
    finally:
        sock.close()
    return None

def _search_for_robot(duration: float = 5.0) -> Optional[Tuple[str, int, int, int]]:
    attempts = max(1, int(duration / DISCOVERY_TIMEOUT))
    return discover_robot(attempts=attempts, timeout=DISCOVERY_TIMEOUT)

def prompt_for_robot_selection(timeout: float = 5.0) -> Optional[Tuple[str, int, int, int]]:
    """Show a blocking window that searches for the robot, allowing manual entry."""
    result: Dict[str, Optional[Tuple[str, int, int, int]]] = {"value": None}

    root = tk.Tk()
    root.title("Robot Discovery")
    root.geometry("400x220")
    root.configure(background=WINDOW_BG)

    status_var = tk.StringVar(value="Searching for robot (up to 5 seconds)...")
    ip_var = tk.StringVar()
    searching = {"active": False}

    frame = ttk.Frame(root, padding=12, style="Panel.TFrame")
    frame.pack(fill="both", expand=True)
    ttk.Label(frame, textvariable=status_var, style="Muted.TLabel", wraplength=360).pack(
        anchor="w", fill="x"
    )

    entry_label = ttk.Label(frame, text="Manual IP address (optional):", style="Subheading.TLabel")
    entry_label.pack(anchor="w", pady=(12, 4))
    entry = ttk.Entry(frame, textvariable=ip_var, style="Console.TEntry")
    entry.pack(fill="x")

    buttons = ttk.Frame(frame, style="Panel.TFrame")
    buttons.pack(fill="x", pady=(16, 0))

    def finish(value: Optional[Tuple[str, int, int, int]]) -> None:
        result["value"] = value
        root.destroy()

    def handle_search_complete(found: Optional[Tuple[str, int, int, int]]) -> None:
        searching["active"] = False
        if found:
            status_var.set(f"Found robot at {found[0]}. Launching GUI...")
            root.after(500, lambda: finish(found))
            return
        status_var.set("Robot not found. Enter an IP or search again for 5 seconds.")
        search_btn.config(state="normal")
        manual_btn.config(state="normal")

    def run_search() -> None:
        if searching["active"]:
            return
        searching["active"] = True
        status_var.set("Searching for robot (up to 5 seconds)...")
        search_btn.config(state="disabled")
        manual_btn.config(state="disabled")

        def worker() -> None:
            found = _search_for_robot(timeout)
            root.after(0, lambda: handle_search_complete(found))

        threading.Thread(target=worker, daemon=True).start()

    def use_manual_ip() -> None:
        raw_ip = ip_var.get().strip()
        if not raw_ip:
            status_var.set("Enter an IP address before selecting manual connect.")
            return
        try:
            ipaddress.ip_address(raw_ip)
        except ValueError:
            status_var.set("Invalid IP address format. Please try again.")
            return
        finish((raw_ip, UDP_DESTINATION[1], COMMAND_DESTINATION[1], TELEMETRY_LISTEN[1]))

    search_btn = ttk.Button(
        buttons,
        text="Search Again (5s)",
        command=run_search,
        style="Accent.TButton",
    )
    search_btn.pack(side="left", expand=True, fill="x")

    manual_btn = ttk.Button(
        buttons,
        text="Use Manual IP",
        command=use_manual_ip,
        style="Accent.TButton",
    )
    manual_btn.pack(side="left", expand=True, fill="x", padx=(8, 0))

    quit_btn = ttk.Button(
        frame,
        text="Cancel",
        command=lambda: finish(None),
        style="Accent.TButton",
    )
    quit_btn.pack(fill="x", pady=(12, 0))

    root.protocol("WM_DELETE_WINDOW", lambda: finish(None))
    run_search()
    entry.focus_set()
    root.mainloop()
    return result["value"]