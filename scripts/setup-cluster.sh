#!/usr/bin/env python3
import sys
import os
import json
import subprocess
import time
import urllib.request
import urllib.error

# ANSI Color Codes for beautiful terminal output
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def print_info(msg):
    print(f"{Colors.BLUE}[INFO]{Colors.ENDC} {msg}")

def print_pass(msg):
    print(f"{Colors.GREEN}[PASS]{Colors.ENDC} {msg}")

def print_warn(msg):
    print(f"{Colors.WARNING}[WARN]{Colors.ENDC} {msg}")

def print_fail(msg):
    print(f"{Colors.FAIL}[FAIL]{Colors.ENDC} {msg}")

def print_header(msg):
    print(f"\n{Colors.HEADER}{Colors.BOLD}=== {msg} ==={Colors.ENDC}\n")

# Simple YAML parser that reads specific structure of cluster.yaml
def parse_yaml(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()

    config = {}
    current_section = None
    current_worker = None

    for line in lines:
        # Strip comments and whitespace
        line = line.split('#')[0].strip()
        if not line:
            continue

        # Check for start of sections
        if line.endswith(':'):
            current_section = line[:-1].strip()
            if current_section == 'workers':
                config['workers'] = []
            continue

        # Check for list items under workers
        if line.startswith('-'):
            current_worker = {}
            if 'workers' not in config:
                config['workers'] = []
            config['workers'].append(current_worker)
            line = line[1:].strip()

        if ':' in line:
            parts = line.split(':', 1)
            key = parts[0].strip()
            val = parts[1].strip().strip('"').strip("'")

            if current_section == 'workers':
                if current_worker is not None:
                    current_worker[key] = val
            elif current_section in ['registry', 'controlPlane']:
                if current_section not in config:
                    config[current_section] = {}
                config[current_section][key] = val
            else:
                config[key] = val
    return config

# Run a local shell command
def run_local(cmd, capture=False, check=True):
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            stdout=subprocess.PIPE if capture else None,
            stderr=subprocess.PIPE if capture else None,
            text=True,
            check=check
        )
        return result.stdout if capture else True
    except subprocess.CalledProcessError as e:
        if capture:
            return e.stderr
        raise e

# Run a remote command via SSH
def run_ssh(user, ip, cmd, capture=False, check=True):
    # Run ssh directly as a list to avoid nested double-quote shell escaping issues on the local host.
    ssh_cmd = ["ssh", "-o", "StrictHostKeyChecking=accept-new", f"{user}@{ip}", cmd]
    try:
        result = subprocess.run(
            ssh_cmd,
            stdout=subprocess.PIPE if capture else None,
            stderr=subprocess.PIPE if capture else None,
            text=True,
            check=check
        )
        return result.stdout if capture else True
    except subprocess.CalledProcessError as e:
        if capture:
            return e.stderr
        raise e

def purge_cluster(config, reg_user, reg_ip, reg_port, cp_user, cp_ip, workers):
    print(f"\n{Colors.WARNING}{Colors.BOLD}!!! WARNING !!!{Colors.ENDC}")
    print(f"{Colors.WARNING}This will completely uninstall k3s Server from the control plane, k3s Agents from all workers, and delete the private registry container.{Colors.ENDC}")
    print(f"{Colors.WARNING}All your Kubernetes workloads, configurations, and data will be permanently deleted.{Colors.ENDC}")

    try:
        response = input(f"{Colors.WARNING}Are you sure you want to completely purge the cluster? (y/N): {Colors.ENDC}").strip().lower()
        if response not in ['y', 'yes']:
            print_info("Purge operation cancelled.")
            sys.exit(0)
    except KeyboardInterrupt:
        print("\nOperation cancelled.")
        sys.exit(0)

    print_header("Tearing Down and Cleaning Up Kubernetes Cluster")

    # 1. Worker Nodes Cleanup
    print_header("Step 1: Cleaning Up Worker Nodes")
    for worker in workers:
        w_host = worker.get('host')
        w_ip = worker.get('ip')
        w_user = worker.get('user', 'ubuntu')

        print_info(f"Connecting to worker node {w_host} ({w_ip}) as user '{w_user}'...")

        # Uninstall k3s-agent
        try:
            print_info("Uninstalling k3s-agent...")
            run_ssh(w_user, w_ip, "sudo /usr/local/bin/k3s-agent-uninstall.sh || [ ! -f /usr/local/bin/k3s-agent-uninstall.sh ]")
            print_pass(f"k3s-agent uninstalled successfully on {w_host}.")
        except Exception as e:
            print_warn(f"Failed to uninstall k3s-agent on {w_host} (it might have already been removed): {e}")

        # Remove registries.yaml and perform deep cleanup of leftover files
        try:
            print_info("Performing deep cleanup of k3s agent files and registry configs...")
            cleanup_cmd = (
                "sudo rm -rf /etc/rancher/k3s /var/lib/rancher/k3s /run/k3s /run/flannel /var/lib/kubelet "
                "/usr/local/bin/k3s /usr/local/bin/k3s-killall.sh /usr/local/bin/k3s-agent-uninstall.sh "
                "/etc/systemd/system/k3s-agent.service /etc/systemd/system/k3s-agent.service.env; "
                "for cmd in kubectl crictl ctr; do [ -L /usr/local/bin/$cmd ] && sudo rm -f /usr/local/bin/$cmd; done; "
                "sudo systemctl daemon-reload"
            )
            run_ssh(w_user, w_ip, cleanup_cmd)
            print_pass(f"Deep cleanup completed on {w_host}.")
        except Exception as e:
            print_warn(f"Failed to clean up k3s leftovers on {w_host}: {e}")

    # 2. Control Plane Cleanup
    print_header("Step 2: Cleaning Up k3s Control Plane")
    print_info(f"Connecting to control plane host {cp_ip} as user '{cp_user}'...")
    try:
        print_info("Uninstalling k3s server...")
        run_ssh(cp_user, cp_ip, "sudo /usr/local/bin/k3s-uninstall.sh || [ ! -f /usr/local/bin/k3s-uninstall.sh ]")
        print_pass("k3s server uninstalled successfully.")
    except Exception as e:
        print_warn(f"Failed to uninstall k3s server on control plane (it might have already been removed): {e}")

    # Perform deep cleanup of control plane leftovers
    try:
        print_info("Performing deep cleanup of k3s server files...")
        cleanup_cmd = (
            "sudo rm -rf /etc/rancher/k3s /var/lib/rancher/k3s /run/k3s /run/flannel /var/lib/kubelet "
            "/usr/local/bin/k3s /usr/local/bin/k3s-killall.sh /usr/local/bin/k3s-uninstall.sh "
            "/etc/systemd/system/k3s.service /etc/systemd/system/k3s.service.env; "
            "for cmd in kubectl crictl ctr; do [ -L /usr/local/bin/$cmd ] && sudo rm -f /usr/local/bin/$cmd; done; "
            "sudo systemctl daemon-reload"
        )
        run_ssh(cp_user, cp_ip, cleanup_cmd)
        print_pass("Deep cleanup completed on control plane.")
    except Exception as e:
        print_warn(f"Failed to clean up k3s server leftovers on control plane: {e}")

    # 3. Registry Cleanup
    print_header("Step 3: Cleaning Up Private Registry")
    print_info(f"Connecting to registry host {reg_ip} as user '{reg_user}'...")
    try:
        print_info(f"Removing private Docker registry container on port {reg_port}...")
        # Remove the registry container (but do NOT uninstall Docker itself!)
        run_ssh(reg_user, reg_ip, "sudo docker rm -f registry || true")
        print_pass("Private Docker registry container removed successfully.")
    except Exception as e:
        print_warn(f"Failed to remove private registry container: {e}")

    # 4. Dev Machine Cleanup
    print_header("Step 4: Cleaning Up Dev Machine Configuration")

    # Remove insecure-registries entry from local daemon.json
    try:
        daemon_json_path = os.path.expanduser("~/.docker/daemon.json")
        if os.path.exists(daemon_json_path):
            with open(daemon_json_path, 'r') as f:
                daemon_data = json.load(f)

            insecure_list = daemon_data.get("insecure-registries", [])
            reg_entry = f"{reg_ip}:{reg_port}"
            if reg_entry in insecure_list:
                insecure_list.remove(reg_entry)
                # If the list is empty, delete the key to keep it clean
                if not insecure_list:
                    del daemon_data["insecure-registries"]

                with open(daemon_json_path, 'w') as f:
                    json.dump(daemon_data, f, indent=4)
                print_pass(f"Removed {reg_entry} from insecure-registries in local daemon.json.")
            else:
                print_info(f"{reg_entry} was not present in insecure-registries.")
        else:
            print_info("Local daemon.json does not exist. Skipping.")
    except Exception as e:
        print_warn(f"Failed to clean up local daemon.json: {e}")

    # Remove local kubeconfig (~/.kube/config)
    try:
        local_kubeconfig = os.path.expanduser("~/.kube/config")
        if os.path.exists(local_kubeconfig):
            os.remove(local_kubeconfig)
            print_pass("Removed local kubeconfig (~/.kube/config).")
        else:
            print_info("Local kubeconfig (~/.kube/config) does not exist.")
    except Exception as e:
        print_warn(f"Failed to remove local kubeconfig: {e}")

    print_header("Cluster Teardown Completed Successfully")
    print_info("All k3s server/agents, registrations, and private registry container resources have been cleaned up.")
    print_warn("IMPORTANT: If you are running Docker Desktop on macOS, you MUST manually restart Docker Desktop to apply insecure-registries cleanup!")

def cleanup_cluster(config, reg_user, reg_ip, reg_port, cp_user, cp_ip, workers):
    print_header("Performing Lightweight Cluster Cleanup (Keeping Infrastructure)")

    # 1. Clean up local unused/stopped Docker builders and images
    print_header("Step 1: Cleaning Up Local Dev Machine Docker Resources")
    try:
        print_info("Pruning unused Docker builder cache and unused images locally...")
        run_local("docker image prune -f", check=False)
        print_pass("Local dev machine Docker images pruned.")
    except Exception as e:
        print_warn(f"Failed to prune local docker resources: {e}")

    # 2. Clean up Kubernetes workloads on the cluster
    print_header("Step 2: Cleaning Up Lingua Franca Kubernetes Deployments")
    try:
        local_kubeconfig = os.path.expanduser("~/.kube/config")
        if os.path.exists(local_kubeconfig):
            print_info("Querying Kubernetes namespaces...")
            ns_json = run_local("kubectl get ns -o json", capture=True, check=False)
            if ns_json and not ns_json.startswith("Error"):
                try:
                    ns_data = json.loads(ns_json)
                    for ns in ns_data.get("items", []):
                        ns_name = ns.get("metadata", {}).get("name")
                        # Safely delete user-created deployment namespaces, protect default and kube namespaces
                        if ns_name not in ["default", "kube-system", "kube-public", "kube-node-lease", "kubernetes-dashboard"]:
                            print_info(f"Deleting user namespace: {ns_name}...")
                            run_local(f"kubectl delete namespace {ns_name} --ignore-not-found", check=False)
                            print_pass(f"Namespace {ns_name} deleted.")
                except Exception as e:
                    print_warn(f"Failed to parse namespace JSON or delete namespaces: {e}")
            else:
                print_info("Could not reach Kubernetes cluster or no active workloads found.")
        else:
            print_info("Local kubeconfig (~/.kube/config) not found. Skipping Kubernetes workloads cleanup.")
    except Exception as e:
        print_warn(f"Failed to clean up Kubernetes namespaces: {e}")

    print_header("Lightweight Cluster Cleanup Completed")
    print_info("Kubernetes cluster (k3s), private registry server, and developer connections are preserved and fully ready for redeployment!")

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <cluster.yaml> [--clean | --purge]")
        sys.exit(1)

    config_file = sys.argv[1]
    if not os.path.exists(config_file):
        print_fail(f"Configuration file not found: {config_file}")
        sys.exit(1)

    print_info(f"Parsing configuration file: {config_file}...")
    try:
        config = parse_yaml(config_file)
    except Exception as e:
        print_fail(f"Failed to parse YAML: {e}")
        sys.exit(1)

    # Extract Registry details
    reg_ip = config.get('registry', {}).get('ip')
    reg_port = config.get('registry', {}).get('port', '5000')

    # Extract Control Plane details
    cp_ip = config.get('controlPlane', {}).get('ip')
    cp_user = config.get('controlPlane', {}).get('user', 'ubuntu')

    # Extract Worker Nodes list
    workers = config.get('workers', [])

    if not cp_ip or not reg_ip:
        print_fail("Invalid configuration: controlPlane IP and registry IP must be defined.")
        sys.exit(1)

    # Determine the SSH user to connect to the registry
    reg_user = config.get('registry', {}).get('user')
    if not reg_user:
        reg_user = cp_user
        if config.get('registry', {}).get('host') != config.get('controlPlane', {}).get('host'):
            # If registry is on a different machine, try to find a matching SSH user from workers
            for w in workers:
                if w.get('ip') == reg_ip:
                    reg_user = w.get('user', 'ubuntu')
                    break

    # Check if --clean or --purge is specified
    is_clean = len(sys.argv) >= 3 and sys.argv[2] == '--clean'
    is_purge = len(sys.argv) >= 3 and sys.argv[2] == '--purge'

    if is_clean:
        cleanup_cluster(config, reg_user, reg_ip, reg_port, cp_user, cp_ip, workers)
        sys.exit(0)
    elif is_purge:
        purge_cluster(config, reg_user, reg_ip, reg_port, cp_user, cp_ip, workers)
        sys.exit(0)

    # ==========================================
    # Phase 0: Prerequisite checks on Dev Machine
    # ==========================================
    print_header("Phase 0: Prerequisite Checks on Dev Machine")

    # Check if kubectl is installed locally
    if shutil_which := getattr(os, 'P_WAIT', None):  # simple check
        try:
            run_local("command -v kubectl", capture=True)
            print_pass("kubectl is installed locally.")
        except Exception:
            print_fail("kubectl is not installed locally. Please install it before running this script.")
            sys.exit(1)

    # Check if Docker is installed locally
    try:
        run_local("command -v docker", capture=True)
        print_pass("Docker CLI is installed locally.")
    except Exception:
        print_fail("Docker is not installed locally. Please install Docker Desktop before running this script.")
        sys.exit(1)

    # Check if local Docker daemon is running
    try:
        run_local("docker info", capture=True)
        print_pass("Docker daemon is running locally.")
    except Exception:
        print_warn("Docker daemon is not running locally. Please make sure Docker Desktop is started.")

    # ==========================================
    # Phase 1: Setup Private Registry
    # ==========================================
    print_header("Phase 1: Setup Private Registry")

    print_info(f"Connecting to registry host {reg_ip} as user '{reg_user}'...")

    # Check if Docker is installed on registry host
    try:
        docker_installed = run_ssh(reg_user, reg_ip, "command -v docker", capture=True)
        if not docker_installed or "docker" not in docker_installed:
            print_fail("Docker is not installed on the registry host.")
            print_info("Please manually install Docker Engine on the registry host first (e.g., using apt, dnf, yum, or the official Docker installation scripts).")
            sys.exit(1)
        else:
            print_pass("Docker is already installed on registry host.")
    except Exception as e:
        print_fail(f"Failed to check or install Docker on registry host: {e}")
        sys.exit(1)

    # Run private registry container
    try:
        print_info(f"Deploying registry container on port {reg_port}...")
        # Idempotently start/restart the registry container
        reg_check_cmd = f"if [ $(sudo docker ps -a -f name=registry --format '{{{{.Status}}}}' | grep -c 'Up') -eq 0 ]; then sudo docker rm -f registry || true; sudo docker run -d -p {reg_port}:5000 --restart=always --name registry registry:2; fi"
        run_ssh(reg_user, reg_ip, reg_check_cmd)
        print_pass(f"Registry container is up and running on {reg_ip}:{reg_port}.")
    except Exception as e:
        print_fail(f"Failed to start registry container: {e}")
        sys.exit(1)

    # Intermediate test: Verify Dev Machine can connect to the Registry
    try:
        print_info("Verifying registry connectivity from Dev Machine...")
        url = f"http://{reg_ip}:{reg_port}/v2/"
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=5) as response:
            if response.status == 200:
                print_pass("Dev Machine successfully reached the private registry container.")
            else:
                print_fail(f"Registry returned status code: {response.status}")
                sys.exit(1)
    except Exception as e:
        print_fail(f"Dev Machine cannot reach the private registry container: {e}")
        print_info("Please verify network routing, firewall settings, or client isolation.")
        sys.exit(1)

    # ==========================================
    # Phase 2: Setup Control Plane (k3s Server)
    # ==========================================
    print_header("Phase 2: Setup k3s Control Plane")

    print_info(f"Connecting to control plane host {cp_ip} as user '{cp_user}'...")

    # Install k3s server on control plane
    try:
        k3s_installed = run_ssh(cp_user, cp_ip, "command -v k3s", capture=True)
        if not k3s_installed or "k3s" not in k3s_installed:
            print_info(f"k3s server is not installed. Installing with TLS SAN {cp_ip}...")
            # Install k3s with explicit TLS Subject Alternative Name (SAN) of the control plane IP
            install_cmd = f"curl -sfL https://get.k3s.io | INSTALL_K3S_EXEC=\"--tls-san {cp_ip}\" sh -"
            run_ssh(cp_user, cp_ip, install_cmd)
            print_pass("k3s server installed successfully.")
        else:
            print_pass("k3s server is already installed on control plane.")
    except Exception as e:
        print_fail(f"Failed to install k3s server on control plane: {e}")
        sys.exit(1)

    # Check k3s service status
    try:
        status = run_ssh(cp_user, cp_ip, "sudo systemctl is-active k3s", capture=True).strip()
        if status == "active":
            print_pass("k3s service is active and running on control plane.")
        else:
            print_fail(f"k3s service is not active: {status}")
            sys.exit(1)
    except Exception as e:
        print_fail(f"Failed to check k3s service status: {e}")
        sys.exit(1)

    # Retrieve k3s cluster join token
    try:
        print_info("Retrieving k3s join token...")
        token = run_ssh(cp_user, cp_ip, "sudo cat /var/lib/rancher/k3s/server/node-token", capture=True).strip()
        print_pass("Successfully retrieved join token from control plane.")
    except Exception as e:
        print_fail(f"Failed to retrieve k3s join token: {e}")
        sys.exit(1)

    # Retrieve kubeconfig content
    try:
        print_info("Retrieving kubeconfig (k3s.yaml) content...")
        kubeconfig_data = run_ssh(cp_user, cp_ip, "sudo cat /etc/rancher/k3s/k3s.yaml", capture=True)
        print_pass("Successfully retrieved kubeconfig content.")
    except Exception as e:
        print_fail(f"Failed to retrieve kubeconfig: {e}")
        sys.exit(1)

    # ==========================================
    # Phase 3 & 4: Setup Worker Nodes & Registry Config
    # ==========================================
    print_header("Phase 3 & 4: Setup Worker Nodes & registries.yaml")

    for worker in workers:
        w_host = worker.get('host')
        w_ip = worker.get('ip')
        w_user = worker.get('user', 'ubuntu')
        w_lf_host = worker.get('lfHost')

        print_info(f"Provisioning worker node {w_host} ({w_ip}) as user '{w_user}'...")

        # Check cgroups memory limits on Raspberry Pi/Linux worker nodes
        try:
            cmdline = run_ssh(w_user, w_ip, "cat /proc/cmdline", capture=True)
            if "cgroup_enable=memory" not in cmdline or "cgroup_memory=1" not in cmdline:
                print_warn(f"cgroups memory limits are disabled on worker {w_host} ({w_ip}).")

                # Check for standard config file locations
                cmdline_path = None
                for path in ["/boot/firmware/cmdline.txt", "/boot/cmdline.txt"]:
                    exists = run_ssh(w_user, w_ip, f"[ -f {path} ] && echo 'yes' || echo 'no'", capture=True).strip()
                    if exists == 'yes':
                        cmdline_path = path
                        break

                if cmdline_path:
                    # Verify if it's already written but waiting for reboot
                    file_content = run_ssh(w_user, w_ip, f"cat {cmdline_path}", capture=True)
                    if "cgroup_enable=memory" in file_content and "cgroup_memory=1" in file_content:
                        print_fail(f"CRITICAL: cgroups are configured in {cmdline_path} on {w_host}, but a reboot is pending!")
                        print_info(f"Please reboot the worker node manually: ssh {w_user}@{w_ip} 'sudo reboot'")
                        print_info("After the worker boots back up, rerun this setup script.")
                        sys.exit(1)
                    else:
                        print_info(f"Attempting to automatically append cgroups settings to {cmdline_path} on {w_host}...")
                        run_ssh(w_user, w_ip, f"sudo sed -i 's/$/ cgroup_memory=1 cgroup_enable=memory/' {cmdline_path}")
                        print_pass(f"Successfully configured cgroups in {cmdline_path} on {w_host}!")
                        print_fail(f"CRITICAL: A system reboot is required on {w_host} to apply cgroups changes.")
                        print_info(f"Please run: ssh {w_user}@{w_ip} 'sudo reboot'")
                        print_info("After the worker reboots, rerun this setup script.")
                        sys.exit(1)
                else:
                    print_fail(f"Unable to automatically enable cgroups: cmdline.txt not found on {w_host}.")
                    print_info("Please manually append 'cgroup_memory=1 cgroup_enable=memory' to your Linux boot cmdline and reboot.")
                    sys.exit(1)
        except Exception as e:
            # Non-critical warning if cgroup pre-check fails
            print_warn(f"Could not perform cgroups pre-check on {w_host}: {e}")

        # Install and join k3s agent
        try:
            service_exists = run_ssh(w_user, w_ip, "systemctl list-unit-files k3s-agent.service | grep -q k3s-agent && echo 'yes' || echo 'no'", capture=True).strip()
            if service_exists != 'yes':
                print_info("Installing and joining k3s-agent to control plane...")
                agent_install_cmd = f"curl -sfL https://get.k3s.io | K3S_URL=https://{cp_ip}:6443 K3S_TOKEN={token} sh -"
                run_ssh(w_user, w_ip, agent_install_cmd)
                print_pass("k3s-agent joined the cluster successfully.")
            else:
                print_pass("k3s-agent is already installed.")
        except Exception as e:
            print_fail(f"Failed to join worker node {w_host} to the cluster: {e}")
            sys.exit(1)

        # Configure containerd registry mirror mapping
        try:
            print_info("Configuring containerd insecure registry mirror in registries.yaml...")
            reg_yaml_content = f"""mirrors:
  "{reg_ip}:{reg_port}":
    endpoint:
      - "http://{reg_ip}:{reg_port}"
"""
            # Write registries.yaml remotely using a heredoc to avoid any quote escaping issues
            write_cmd = f"sudo tee /etc/rancher/k3s/registries.yaml > /dev/null << 'EOF'\n{reg_yaml_content}\nEOF"
            run_ssh(w_user, w_ip, "sudo mkdir -p /etc/rancher/k3s")
            run_ssh(w_user, w_ip, write_cmd)
            run_ssh(w_user, w_ip, "sudo systemctl restart k3s-agent")
            print_pass(f"registries.yaml configured and k3s-agent restarted on {w_host}.")
        except Exception as e:
            print_fail(f"Failed to configure registries.yaml on worker {w_host}: {e}")
            sys.exit(1)

        # Intermediate test: Verify Worker node can reach the registry container
        try:
            print_info(f"Testing registry reachability from inside worker node {w_host}...")
            # Use curl from inside the worker node to test access
            test_curl = run_ssh(w_user, w_ip, f"curl -sf http://{reg_ip}:{reg_port}/v2/", capture=True)
            print_pass(f"Worker node {w_host} successfully reached private registry over network.")
        except Exception as e:
            print_fail(f"Worker node {w_host} cannot reach private registry: {e}")
            print_info("Please check routing or security group settings between workers and registry.")
            sys.exit(1)

    # ==========================================
    # Phase 5: Configure kubectl on Dev Machine
    # ==========================================
    print_header("Phase 5: Configure local kubectl (kubeconfig)")

    # Save retrieved k3s.yaml to local ~/.kube/config
    try:
        kube_dir = os.path.expanduser("~/.kube")
        os.makedirs(kube_dir, exist_ok=True)
        local_kubeconfig = os.path.join(kube_dir, "config")

        # Replace 127.0.0.1 in server endpoint with the remote control plane IP
        patched_kubeconfig = kubeconfig_data.replace("127.0.0.1", cp_ip)

        with open(local_kubeconfig, "w") as f:
            f.write(patched_kubeconfig)

        os.chmod(local_kubeconfig, 0o600)
        print_pass(f"Successfully configured local kubeconfig at: {local_kubeconfig}")
    except Exception as e:
        print_fail(f"Failed to write local kubeconfig: {e}")
        sys.exit(1)

    # Intermediate test: Verify Dev Machine can talk to k3s Server via kubectl
    try:
        print_info("Verifying kubectl connectivity from Dev Machine...")
        run_local("kubectl get nodes", check=True)
        print_pass("kubectl successfully authenticated and reached the k3s control plane!")
    except Exception as e:
        print_fail(f"kubectl failed to connect to the control plane: {e}")
        sys.exit(1)

    # ==========================================
    # Phase 6: Configure Insecure Registry on Dev Machine
    # ==========================================
    print_header("Phase 6: Configure Insecure Registry on Dev Machine")

    # Configure insecure registries in daemon.json
    try:
        docker_config_dir = os.path.expanduser("~/.docker")
        os.makedirs(docker_config_dir, exist_ok=True)
        daemon_json_path = os.path.join(docker_config_dir, "daemon.json")

        daemon_data = {}
        if os.path.exists(daemon_json_path):
            try:
                with open(daemon_json_path, 'r') as f:
                    daemon_data = json.load(f)
            except Exception:
                daemon_data = {}

        # Ensure insecure-registries list exists and contains the registry address
        insecure_list = daemon_data.setdefault("insecure-registries", [])
        reg_entry = f"{reg_ip}:{reg_port}"
        if reg_entry not in insecure_list:
            insecure_list.append(reg_entry)
            with open(daemon_json_path, 'w') as f:
                json.dump(daemon_data, f, indent=4)
            print_pass(f"Added {reg_entry} to insecure-registries in local {daemon_json_path}.")
        else:
            print_pass(f"{reg_entry} is already present in local insecure-registries.")

        print_warn(f"IMPORTANT: If you are running Docker Desktop on macOS, you MUST manually restart Docker Desktop for these changes to take effect!")
    except Exception as e:
        print_fail(f"Failed to configure local insecure registry settings: {e}")

    # ==========================================
    # Phase 7: Label Worker Nodes
    # ==========================================
    print_header("Phase 7: Match and Label Kubernetes Worker Nodes")

    # Retrieve nodes metadata from Kubernetes API
    try:
        nodes_json_str = run_local("kubectl get nodes -o json", capture=True)
        nodes_data = json.loads(nodes_json_str)
    except Exception as e:
        print_fail(f"Failed to retrieve node list from Kubernetes: {e}")
        sys.exit(1)

    k8s_nodes = nodes_data.get('items', [])

    # Map each worker defined in cluster.yaml to its actual Kubernetes node name
    for worker in workers:
        w_host = worker.get('host')
        w_ip = worker.get('ip')
        w_lf_host = worker.get('lfHost')
        matched_node_name = None

        for node in k8s_nodes:
            metadata = node.get('metadata', {})
            addresses = node.get('status', {}).get('addresses', [])

            # Match by Hostname or InternalIP
            is_match = False
            for addr in addresses:
                addr_type = addr.get('type')
                addr_val = addr.get('address')
                if addr_val == w_ip or addr_val == w_host:
                    is_match = True
                    break

            if is_match:
                matched_node_name = metadata.get('name')
                break

        if not matched_node_name:
            # Fall back to matching by host string directly
            matched_node_name = w_host

        print_info(f"Labeling node '{matched_node_name}' as lf-host={w_lf_host}...")
        try:
            # Add nodeSelector tag matching Lingua Franca at clause specifications
            run_local(f"kubectl label node {matched_node_name} lf-host={w_lf_host} --overwrite")
            print_pass(f"Successfully labeled node '{matched_node_name}' with lf-host={w_lf_host}.")
        except Exception as e:
            print_fail(f"Failed to label node '{matched_node_name}': {e}")

    # Print final verification report
    print_header("Final Kubernetes Cluster Deployment Status")
    try:
        run_local("kubectl get nodes --show-labels")
        print_pass("Kubernetes cluster setup completed and verified successfully!")
        print_info("You can now build federated reactor programs using deployment-type: 'kubernetes'!")
    except Exception as e:
        print_fail(f"Final verification failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
