#!/usr/bin/env python3

"""
Scan Devices
=====================

This script is to detect all devices on the local network and print their information
including IP address, MAC address, and hostname.

Usage
-----------------

To use this script, run it with the following command:

```
python scan_devices.py -i <interface> -r <network_range> -o <output_file>
```

where:
- `<interface>`: The network interface to scan (optional). If not specified, all interfaces will be scanned.
- `<network_range>`: The range of network (optinal).
- `<output_file>`: The file to save the results (optional).
"""

import sys
import os
import time
import argparse
import socket
import subprocess
import re
from datetime import datetime
import ipaddress
import netifaces
import shutil


def get_network_interfaces():
    """Get all network interfaces and their IP addresses, ignoring can0 and can1 interfaces"""
    interfaces = {}
    # List of interfaces to ignore
    ignore_interfaces = ["can0", "can1", "lo"]

    for iface in netifaces.interfaces():
        # Skip interfaces that should be ignored
        if iface in ignore_interfaces:
            continue

        try:
            addrs = netifaces.ifaddresses(iface)
            if netifaces.AF_INET in addrs:
                ip_info = addrs[netifaces.AF_INET][0]
                if "addr" in ip_info:
                    interfaces[iface] = {
                        "ip": ip_info["addr"],
                        "netmask": ip_info.get("netmask", ""),
                    }
        except ValueError:
            continue
    return interfaces


def get_network_range(ip, netmask):
    """Calculate network range based on IP and subnet mask"""
    try:
        network = ipaddress.IPv4Network(f"{ip}/{netmask}", strict=False)
        return str(network)
    except ValueError as e:
        print(f"Error: Cannot calculate network range - {e}")
        return None


def which(cmd):
    """Check if a command exists"""
    return shutil.which(cmd) is not None


def get_arp_table():
    """Get all entries from the system ARP table"""
    # Check if arp command is available
    if not which("arp"):
        print("arp command not available")
        return ""

    try:
        # Run arp -a command to get ARP table
        result = subprocess.run(
            ["arp", "-a"], capture_output=True, text=True, check=True
        )
        return result.stdout
    except subprocess.SubprocessError as e:
        print(f"Error running arp command: {e}")
        return ""
    except FileNotFoundError:
        print("arp command not installed on the system")
        return ""


def ping_network(network_range):
    """
    Scan devices on the network using ping, which helps update the ARP table
    """
    try:
        network = ipaddress.IPv4Network(network_range)
        print(f"Scanning network {network_range}...")

        # Only scan the first 254 IP addresses to avoid scanning too large networks
        host_count = min(
            254, network.num_addresses - 2
        )  # Subtract network and broadcast addresses

        for i, ip in enumerate(network.hosts()):
            if i >= host_count:
                break

            ip_str = str(ip)
            # Send one ping packet with 1 second timeout
            ping_cmd = ["ping", "-c", "1", "-W", "1", ip_str]
            subprocess.Popen(
                ping_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

            # Print progress
            if i % 10 == 0:
                sys.stdout.write(f"\rPing scanning... {i}/{host_count}")
                sys.stdout.flush()

        sys.stdout.write(f"\rPing scanning... {host_count}/{host_count}\n")
        sys.stdout.flush()

        # Wait for ARP table to update
        time.sleep(2)

    except Exception as e:
        print(f"Error during ping scan: {e}")


def parse_arp_table(arp_output, network_range=None):
    """
    Parse the output of arp command, extract IP and MAC addresses

    If network_range is provided, only return devices within that range
    """
    devices = []
    network = None
    if network_range:
        try:
            network = ipaddress.IPv4Network(network_range)
        except ValueError:
            print(f"Invalid network range: {network_range}")

    # Match entries in the ARP table, supporting different output formats
    # Linux format: "hostname.domain (192.168.1.1) at 00:11:22:33:44:55 [ether] on eth0"
    # Or: "? (192.168.1.1) at 00:11:22:33:44:55 [ether] on eth0"
    pattern = r"\(?([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)\)?\s+at\s+([0-9a-fA-F:]+)"

    matches = re.findall(pattern, arp_output)
    for ip, mac in matches:
        # If network range is specified, check if IP is in range
        if network and ipaddress.IPv4Address(ip) not in network:
            continue

        # Exclude invalid MAC addresses
        if mac.lower() == "ff:ff:ff:ff:ff:ff" or mac.lower() == "00:00:00:00:00:00":
            continue

        devices.append({"ip": ip, "mac": mac})

    return devices


def get_ip_neighbors():
    """
    Get output of the Linux 'ip neigh' command (for modern Linux systems)
    """
    # Check if ip command is available
    if not which("ip"):
        print("ip command not available")
        return ""

    try:
        result = subprocess.run(
            ["ip", "neigh", "show"], capture_output=True, text=True, check=True
        )
        return result.stdout
    except subprocess.SubprocessError:
        print("Error running ip neigh command")
        return ""
    except FileNotFoundError:
        print("ip command not installed on the system")
        return ""


def read_proc_arp():
    """
    Read ARP table directly from /proc/net/arp
    Suitable for Linux systems
    """
    devices = []
    arp_file = "/proc/net/arp"

    if not os.path.exists(arp_file):
        print(f"{arp_file} does not exist")
        return devices

    try:
        with open(arp_file, "r") as f:
            # Skip header line
            next(f)
            for line in f:
                parts = line.split()
                if len(parts) >= 4:
                    ip = parts[0]
                    mac = parts[3]

                    # Exclude invalid MAC addresses
                    if (
                        mac.lower() == "ff:ff:ff:ff:ff:ff"
                        or mac.lower() == "00:00:00:00:00:00"
                    ):
                        continue

                    devices.append({"ip": ip, "mac": mac})
    except Exception as e:
        print(f"Error reading {arp_file}: {e}")

    return devices


def parse_ip_neighbors(neigh_output, network_range=None):
    """
    Parse output of 'ip neigh' command
    Format: "192.168.1.1 dev eth0 lladdr 00:11:22:33:44:55 REACHABLE"
    """
    devices = []
    network = None
    if network_range:
        try:
            network = ipaddress.IPv4Network(network_range)
        except ValueError:
            print(f"Invalid network range: {network_range}")

    # Match ip neigh output
    pattern = r"([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+).*?\s+lladdr\s+([0-9a-fA-F:]+)"

    matches = re.findall(pattern, neigh_output)
    for ip, mac in matches:
        # If network range is specified, check if IP is in range
        if network and ipaddress.IPv4Address(ip) not in network:
            continue

        # Exclude invalid MAC addresses
        if mac.lower() == "ff:ff:ff:ff:ff:ff" or mac.lower() == "00:00:00:00:00:00":
            continue

        devices.append({"ip": ip, "mac": mac})

    return devices


def get_hostname(ip):
    """Try to get hostname corresponding to IP address"""
    try:
        return socket.gethostbyaddr(ip)[0]
    except socket.herror:
        return "Unknown hostname"


def format_mac(mac):
    """Format MAC address"""
    return mac.upper()


def print_devices(devices):
    """Print device information"""
    if not devices:
        print("No devices found")
        return

    print(f"\nFound {len(devices)} devices:\n")
    print(f"{'IP Address':<16} {'MAC Address':<18} {'Interface':<10} {'Hostname'}")
    print("-" * 75)

    # Sort by IP address
    devices.sort(key=lambda x: [int(part) for part in x["ip"].split(".")])

    for device in devices:
        hostname = get_hostname(device["ip"])
        interface = device.get("interface", "Unknown")
        print(
            f"{device['ip']:<16} {format_mac(device['mac']):<18} {interface:<10} {hostname}"
        )


def scan_network(network_range=None, interface=None):
    """
    Scan devices on the network

    Parameters:
        network_range: Network range to scan (e.g., '192.168.1.0/24')
        interface: Network interface to use

    Returns:
        List of found devices, each containing IP and MAC address
    """
    # First run ping scan to update ARP table
    if network_range:
        ping_network(network_range)

    # Get ARP table
    devices = []
    methods_tried = 0

    # Method 1: Try using ip neigh command (modern Linux systems)
    print("Trying to get network devices using 'ip neigh' command...")
    neigh_output = get_ip_neighbors()
    if neigh_output:
        devices = parse_ip_neighbors(neigh_output, network_range)
        methods_tried += 1

    # Method 2: If ip neigh command has no results, try using arp -a command
    if not devices and methods_tried < 3:
        print("Trying to get network devices using 'arp -a' command...")
        arp_output = get_arp_table()
        if arp_output:
            devices = parse_arp_table(arp_output, network_range)
            methods_tried += 1

    # Method 3: Read directly from /proc/net/arp (Linux specific)
    if not devices and methods_tried < 3:
        print("Trying to read network devices from /proc/net/arp...")
        proc_devices = read_proc_arp()
        if proc_devices:
            # If network range is specified, filter devices
            if network_range:
                network = ipaddress.IPv4Network(network_range)
                devices = [
                    d for d in proc_devices if ipaddress.IPv4Address(d["ip"]) in network
                ]
            else:
                devices = proc_devices
            methods_tried += 1

    if methods_tried == 0:
        print("Warning: All methods to get ARP information failed")
        print(
            "Please make sure you have permissions to access network information or necessary network tools are installed"
        )

    # Deduplicate (may have duplicate entries)
    unique_devices = []
    seen_ips = set()
    for device in devices:
        if device["ip"] not in seen_ips:
            seen_ips.add(device["ip"])
            # Add interface information to the device
            device["interface"] = interface
            unique_devices.append(device)

    return unique_devices


def main():
    parser = argparse.ArgumentParser(description="Scan devices on the local network")
    parser.add_argument("-i", "--interface", help="Network interface to use")
    parser.add_argument(
        "-r", "--range", help="Network range to scan (e.g., 192.168.1.0/24)"
    )
    parser.add_argument("-o", "--output", help="Save results to file")
    args = parser.parse_args()

    start_time = time.time()
    all_devices = []

    # Get network range
    if args.range:
        # If network range is specified, use it directly
        network_range = args.range
        print(f"Using specified network range: {network_range}")

        # Run scan
        devices = scan_network(network_range, args.interface)
        all_devices.extend(devices)
    else:
        # Get all network interfaces
        interfaces = get_network_interfaces()
        if not interfaces:
            print("Error: No active network interfaces found")
            sys.exit(1)

        if args.interface:
            # If interface is specified, only scan that interface
            if args.interface not in interfaces:
                print(f"Error: Interface {args.interface} not found")
                print("Available interfaces:", ", ".join(interfaces.keys()))
                sys.exit(1)

            # Scan specified interface
            interface_name = args.interface
            ip = interfaces[interface_name]["ip"]
            netmask = interfaces[interface_name]["netmask"]
            network_range = get_network_range(ip, netmask)

            if not network_range:
                sys.exit(1)

            print(f"Using interface: {interface_name} ({ip})")
            devices = scan_network(network_range, interface_name)
            all_devices.extend(devices)
        else:
            # If no interface is specified, scan all interfaces
            print(f"Found {len(interfaces)} network interfaces, scanning all...")

            for interface_name, interface_info in interfaces.items():
                ip = interface_info["ip"]
                netmask = interface_info["netmask"]
                network_range = get_network_range(ip, netmask)

                if not network_range:
                    continue

                print(f"\nScanning interface: {interface_name} ({ip})")
                devices = scan_network(network_range, interface_name)
                all_devices.extend(devices)

    # Deduplicate (multiple interfaces may discover the same devices)
    unique_devices = []
    seen_ips = set()
    for device in all_devices:
        if device["ip"] not in seen_ips:
            seen_ips.add(device["ip"])
            unique_devices.append(device)

    # Print results
    print_devices(unique_devices)

    # Show scan time
    scan_time = time.time() - start_time
    print(f"\nScan completed, time elapsed: {scan_time:.2f} seconds")

    # Save results to file (if specified)
    if args.output and unique_devices:
        try:
            with open(args.output, "w") as f:
                f.write(f"Scan time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                if args.range:
                    f.write(f"Network range: {args.range}\n\n")
                else:
                    f.write("Scanned all network interfaces\n\n")

                f.write(
                    f"{'IP Address':<16} {'MAC Address':<18} {'Interface':<10} {'Hostname'}\n"
                )
                f.write("-" * 75 + "\n")
                for device in unique_devices:
                    hostname = get_hostname(device["ip"])
                    interface = device.get("interface", "Unknown")
                    f.write(
                        f"{device['ip']:<16} {format_mac(device['mac']):<18} {interface:<10} {hostname}\n"
                    )
                f.write(
                    f"\nScan completed, found {len(unique_devices)} devices, time elapsed: {scan_time:.2f} seconds"
                )
            print(f"\nResults saved to: {args.output}")
        except Exception as e:
            print(f"Error saving file: {e}")


if __name__ == "__main__":
    main()
